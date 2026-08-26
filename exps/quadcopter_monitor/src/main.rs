// RusTReach as a runtime monitor for the Vanderbilt ANSR stack (replay bin).
//
// Replays a recorded VU-stack flight (CSV from the run's mcap actual_pose,
// NED -> x=north, y=east, z=altitude-up) and, at each monitor step, computes
// an anytime reach set of the DJI-F450 quadcopter model over the next HORIZON
// seconds under a goal-conditioned controller headed at the recorded
// near-term position. Reach boxes are checked in 3-D against the city
// elevation grid (the same city_1000_1000.npz the VU world_model obstacle_map
// node publishes). Core logic lives in lib.rs (shared with the
// quad_reach_monitor_py maturin wheel).
//
// Usage:
//   quadcopter_monitor <elev.f32> <nx> <ny> <res> <origin_x> <origin_y> \
//                      <monitor_input.csv> <out_dir> \
//                      [wall_ms=100] [horizon_s=2.0] [lat_margin=0.5] \
//                      [vert_margin=1.0] [start_idx=0] [pos_unc=0.0] \
//                      [vel_unc=0.0] [z_bias=0.0]
//
// pos_unc / vel_unc: half-widths of the initial state-estimate uncertainty box
//   (m, m/s) applied to x,y,z / vx,vy,vz — a runtime monitor for a fielded
//   stack should account for localization error.
// z_bias: additive altitude-sensor fault injected into the state AND goal fed
//   to the monitor (m; negative = vehicle believed higher than it is /
//   monitor sees it lower). Used for a clearly-labeled fault-injection pass.
//
// monitor_input.csv columns: idx,t,x,y,z,vx,vy,vz,gx,gy,gz  (header row)
// Outputs (appended incrementally, flushed per step so a mid-run abort keeps
// completed steps):
//   <out_dir>/monitor_steps.csv : idx,t,safe,wall_us,iterations,n_rects,
//                                 x,y,z,vx,vy,vz,gx,gy,gz
//   <out_dir>/monitor_rects.csv : idx,time,xmin,xmax,ymin,ymax,zmin,zmax
//                                 (per-0.05 s-bin hulls; verdict used raw boxes)

use std::env;
use std::fs;
use std::fs::File;
use std::io::Write;

use quadcopter_monitor::{bin_hulls, check_segment, set_city, City, NUM_DIMS};

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 9 {
        eprintln!(
            "Usage: {} <elev.f32> <nx> <ny> <res> <origin_x> <origin_y> \
             <monitor_input.csv> <out_dir> [wall_ms] [horizon_s] [lat_margin] \
             [vert_margin] [start_idx] [pos_unc] [vel_unc] [z_bias]",
            args[0]
        );
        std::process::exit(1);
    }
    let elev_path = &args[1];
    let nx: usize = args[2].parse().expect("nx");
    let ny: usize = args[3].parse().expect("ny");
    let res: f64 = args[4].parse().expect("res");
    let ox: f64 = args[5].parse().expect("origin_x");
    let oy: f64 = args[6].parse().expect("origin_y");
    let input_csv = &args[7];
    let out_dir = &args[8];
    let wall_ms: u64 = args.get(9).map(|s| s.parse().unwrap()).unwrap_or(100);
    let horizon: f64 = args.get(10).map(|s| s.parse().unwrap()).unwrap_or(2.0);
    let lat_margin: f64 = args.get(11).map(|s| s.parse().unwrap()).unwrap_or(0.5);
    let vert_margin: f64 = args.get(12).map(|s| s.parse().unwrap()).unwrap_or(1.0);
    let start_idx: usize = args.get(13).map(|s| s.parse().unwrap()).unwrap_or(0);
    let pos_unc: f64 = args.get(14).map(|s| s.parse().unwrap()).unwrap_or(0.0);
    let vel_unc: f64 = args.get(15).map(|s| s.parse().unwrap()).unwrap_or(0.0);
    let z_bias: f64 = args.get(16).map(|s| s.parse().unwrap()).unwrap_or(0.0);

    // Load city elevation grid
    let raw = fs::read(elev_path).expect("read elev file");
    assert_eq!(raw.len(), nx * ny * 4, "elev file size mismatch");
    let mut elev = vec![0f32; nx * ny];
    for (k, ch) in raw.chunks_exact(4).enumerate() {
        elev[k] = f32::from_le_bytes([ch[0], ch[1], ch[2], ch[3]]);
    }
    set_city(City { nx, ny, res, ox, oy, elev, lat_margin, vert_margin });
    println!(
        "Loaded city grid {}x{} res {} origin ({}, {}); lat_margin {} vert_margin {} \
         pos_unc {} vel_unc {} z_bias {}",
        nx, ny, res, ox, oy, lat_margin, vert_margin, pos_unc, vel_unc, z_bias
    );

    fs::create_dir_all(out_dir).expect("create out dir");
    let append = start_idx > 0;
    let mut steps_f = File::options()
        .create(true)
        .append(append)
        .write(true)
        .truncate(!append)
        .open(format!("{}/monitor_steps.csv", out_dir))
        .unwrap();
    let mut rects_f = File::options()
        .create(true)
        .append(append)
        .write(true)
        .truncate(!append)
        .open(format!("{}/monitor_rects.csv", out_dir))
        .unwrap();
    if !append {
        writeln!(steps_f, "idx,t,safe,wall_us,iterations,n_rects,x,y,z,vx,vy,vz,gx,gy,gz").unwrap();
        writeln!(rects_f, "idx,time,xmin,xmax,ymin,ymax,zmin,zmax").unwrap();
    }

    let mut rdr = csv::Reader::from_path(input_csv).expect("open monitor input csv");
    let mut n_unsafe = 0usize;
    let mut n_steps = 0usize;
    let mut wall_sum_us = 0f64;
    let mut wall_max_us = 0f64;

    for rec in rdr.records() {
        let rec = rec.unwrap();
        let idx: usize = rec[0].parse().unwrap();
        if idx < start_idx {
            continue;
        }
        let t: f64 = rec[1].parse().unwrap();
        let f = |k: usize| -> f64 { rec[k].parse().unwrap() };
        let (x, y, z, vx, vy, vz) = (f(2), f(3), f(4) + z_bias, f(5), f(6), f(7));
        let (gx, gy, gz) = (f(8), f(9), f(10) + z_bias);

        let mut state = [0.0f64; NUM_DIMS];
        state[0] = x;
        state[1] = y;
        state[2] = z;
        // phi, theta, psi ~ 0 (linearized model; translation decoupled from yaw)
        state[6] = vx;
        state[7] = vy;
        state[8] = vz;

        let res = check_segment(state, [gx, gy, gz], horizon, wall_ms, pos_unc, vel_unc);

        writeln!(
            steps_f,
            "{},{:.3},{},{:.0},{},{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3}",
            idx, t, if res.safe { 1 } else { 0 }, res.wall_us, res.iterations,
            res.storage.len(), x, y, z, vx, vy, vz, gx, gy, gz
        )
        .unwrap();
        for (bt, r) in bin_hulls(&res.storage, horizon, 0.05) {
            writeln!(
                rects_f,
                "{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4}",
                idx, bt,
                r.dims[0].min, r.dims[0].max,
                r.dims[1].min, r.dims[1].max,
                r.dims[2].min, r.dims[2].max
            )
            .unwrap();
        }
        steps_f.flush().unwrap();
        rects_f.flush().unwrap();

        n_steps += 1;
        if !res.safe {
            n_unsafe += 1;
        }
        wall_sum_us += res.wall_us;
        if res.wall_us > wall_max_us {
            wall_max_us = res.wall_us;
        }
    }

    println!(
        "monitor done: {} steps, {} unsafe, wall avg {:.1} ms, max {:.1} ms (budget {} ms)",
        n_steps,
        n_unsafe,
        wall_sum_us / n_steps.max(1) as f64 / 1000.0,
        wall_max_us / 1000.0,
        wall_ms
    );
}
