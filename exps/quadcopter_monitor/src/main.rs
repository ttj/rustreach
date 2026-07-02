// RusTReach as a runtime monitor for the Vanderbilt ANSR stack.
//
// Replays a recorded VU-stack flight (scenario-007 camrun, NED actual_pose from
// the mcap bag, converted to x=north, y=east, z=altitude-up) and, at each
// monitor step, computes an anytime reach set of the DJI-F450 quadcopter model
// over the next HORIZON seconds under a goal-conditioned controller headed at
// the recorded near-term position. The reach boxes are checked in 3-D against
// the city elevation grid (the same city_1000_1000.npz the VU world_model
// obstacle_map node publishes): a box is UNSAFE if any city cell overlapped by
// its (bloated) x-y extent has elevation >= box_alt_min - vertical margin.
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

use std::env;
use std::fs;
use std::fs::File;
use std::io::Write;
use std::sync::Mutex;
use std::time::Instant;

use lazy_static::lazy_static;

use rtreach::face_lift::{face_lifting_iterative_improvement, LiftingSettings, ITERATIONS_AT_QUIT};
use rtreach::geometry::{hyperrectangle_grow_to_convex_hull, HyperRectangle};

use quadcopter::controller::model_sample_action;
use quadcopter::dynamics_quadcopter::{QuadcopterModel, QUAD_NUM_DIMS as NUM_DIMS};

const QUAD_HALF_WIDTH: f64 = 0.16; // m, same bloat as quadcopter_model.rs

struct City {
    nx: usize,
    ny: usize,
    res: f64,
    ox: f64,
    oy: f64,
    elev: Vec<f32>, // row-major [ix][iy]
    lat_margin: f64,
    vert_margin: f64,
}

lazy_static! {
    static ref CITY: Mutex<Option<City>> = Mutex::new(None);
}

/// 3-D check of one reach box against the city elevation grid.
/// Returns true if SAFE.
fn city_box_safe(r: &HyperRectangle<NUM_DIMS>) -> bool {
    let guard = CITY.lock().unwrap();
    let city = match guard.as_ref() {
        Some(c) => c,
        None => return true,
    };
    let bloat = QUAD_HALF_WIDTH + city.lat_margin;
    let xmin = r.dims[0].min - bloat;
    let xmax = r.dims[0].max + bloat;
    let ymin = r.dims[1].min - bloat;
    let ymax = r.dims[1].max + bloat;
    let alt_floor = r.dims[2].min - city.vert_margin; // altitude-up frame

    let i0 = (((xmin - city.ox) / city.res).floor().max(0.0)) as usize;
    let i1 = (((xmax - city.ox) / city.res).ceil().min(city.nx as f64 - 1.0)) as usize;
    let j0 = (((ymin - city.oy) / city.res).floor().max(0.0)) as usize;
    let j1 = (((ymax - city.oy) / city.res).ceil().min(city.ny as f64 - 1.0)) as usize;
    for i in i0..=i1 {
        let row = i * city.ny;
        for j in j0..=j1 {
            if city.elev[row + j] as f64 >= alt_floor {
                return false;
            }
        }
    }
    true
}

fn monitor_intermediate_state(
    r: &mut HyperRectangle<NUM_DIMS>,
    time: f64,
    _obstacle_sim_fn: fn(f64, &mut Vec<Vec<Vec<f64>>>),
    store_rect: bool,
    storage_vec: &mut Vec<(f64, HyperRectangle<NUM_DIMS>)>,
) -> bool {
    if store_rect {
        storage_vec.push((time, *r));
    }
    city_box_safe(r)
}

// Final-time callback: same safety check, but never stores. The anytime loop's
// quit path calls this with the whole-horizon hull, which would otherwise
// append a spurious giant box to the storage vector.
fn monitor_final_state(
    r: &mut HyperRectangle<NUM_DIMS>,
    _time: f64,
    _obstacle_sim_fn: fn(f64, &mut Vec<Vec<Vec<f64>>>),
    _store_rect: bool,
    _storage_vec: &mut Vec<(f64, HyperRectangle<NUM_DIMS>)>,
) -> bool {
    city_box_safe(r)
}

fn monitor_restarted(_store_rect: bool, storage_vec: &mut Vec<(f64, HyperRectangle<NUM_DIMS>)>) {
    storage_vec.truncate(1);
}

fn obstacle_sim_noop(_: f64, _: &mut Vec<Vec<Vec<f64>>>) {}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 9 {
        eprintln!(
            "Usage: {} <elev.f32> <nx> <ny> <res> <origin_x> <origin_y> \
             <monitor_input.csv> <out_dir> [wall_ms] [horizon_s] [lat_margin] \
             [vert_margin] [start_idx]",
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
    {
        let mut city = CITY.lock().unwrap();
        *city = Some(City { nx, ny, res, ox, oy, elev, lat_margin, vert_margin });
    }
    println!(
        "Loaded city grid {}x{} res {} origin ({}, {}); lat_margin {} vert_margin {} \
         pos_unc {} vel_unc {} z_bias {}",
        nx, ny, res, ox, oy, lat_margin, vert_margin, pos_unc, vel_unc, z_bias
    );

    // Quadcopter model with goal-conditioned fixed controller (no ONNX model:
    // model_sample_action falls back to goal_conditioned_sample_action).
    let mut quad = QuadcopterModel::default();
    quad.set_ctrl_fn(model_sample_action);

    // Reachability parameters (match the corr/nbd experiments)
    let init_step_size = 0.1;
    let store_rect = true;
    let fixed_step = false;
    let dynamic_control = true;

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
        let (x, y, mut z, vx, vy, vz) = (f(2), f(3), f(4), f(5), f(6), f(7));
        let (gx, gy, mut gz) = (f(8), f(9), f(10));
        z += z_bias;
        gz += z_bias;

        let mut state = [0.0f64; NUM_DIMS];
        state[0] = x;
        state[1] = y;
        state[2] = z;
        // phi, theta, psi ~ 0 (linearized model; translation decoupled from yaw)
        state[6] = vx;
        state[7] = vy;
        state[8] = vz;

        quad.set_goal([gx, gy, gz]);
        let ctrl_input = quad.sample_state_action(&state).to_vec();

        let mut set: LiftingSettings<NUM_DIMS> = LiftingSettings::<NUM_DIMS> {
            init: HyperRectangle::default(),
            reach_time: horizon,
            initial_step_size: init_step_size,
            max_rect_width_before_error: 100.0,
            max_runtime_milliseconds: wall_ms,
            obstacle_sim_fn: obstacle_sim_noop,
            reached_at_intermediate_time: Some(monitor_intermediate_state),
            reached_at_final_time: Some(monitor_final_state),
            restarted_computation: Some(monitor_restarted),
        };
        for d in 0..NUM_DIMS {
            set.init.dims[d].min = state[d];
            set.init.dims[d].max = state[d];
        }
        for d in 0..3 {
            set.init.dims[d].min -= pos_unc;
            set.init.dims[d].max += pos_unc;
            set.init.dims[d + 6].min -= vel_unc;
            set.init.dims[d + 6].max += vel_unc;
        }
        let mut storage: Vec<(f64, HyperRectangle<NUM_DIMS>)> = Vec::new();
        storage.push((0.0, set.init));

        // Degenerate case: exactly-zero dynamics (hover with frozen pose and a
        // goal equal to the state). All derivative bounds are identically zero,
        // so the reach set is exactly the initial point for the whole horizon.
        // The upstream anytime loop would halve its step size to <1e-7 on this
        // input (every iteration terminates instantly) and conservatively
        // return unsafe from the "step size too small" path; short-circuit it
        // with the exact answer instead.
        let degenerate = pos_unc == 0.0
            && vel_unc == 0.0
            && vx.abs() < 1e-9
            && vy.abs() < 1e-9
            && vz.abs() < 1e-9
            && ctrl_input.iter().all(|c| c.abs() < 1e-9);

        let t_start = Instant::now();
        let (safe, iterations) = if degenerate {
            let mut point_rect = set.init;
            let s = city_box_safe(&point_rect);
            hyperrectangle_grow_to_convex_hull(&mut point_rect, &set.init);
            storage.push((horizon, point_rect));
            (s, 0)
        } else {
            let s = face_lifting_iterative_improvement(
                &quad,
                0,
                &mut set,
                &ctrl_input,
                store_rect,
                &mut storage,
                fixed_step,
                dynamic_control,
            );
            (s, *ITERATIONS_AT_QUIT.lock().unwrap())
        };
        let wall_us = t_start.elapsed().as_micros() as f64;

        writeln!(
            steps_f,
            "{},{:.3},{},{:.0},{},{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3}",
            idx, t, if safe { 1 } else { 0 }, wall_us, iterations, storage.len(),
            x, y, z, vx, vy, vz, gx, gy, gz
        )
        .unwrap();
        // The finest anytime iteration stores tens of thousands of tiny
        // 2-step hulls; compress to per-time-bin convex hulls for output
        // (display only — the safety verdict above used every raw box).
        let bin_w = 0.05f64;
        let n_bins = (horizon / bin_w).ceil() as usize + 1;
        let mut bins: Vec<Option<(f64, HyperRectangle<NUM_DIMS>)>> = vec![None; n_bins];
        for (rt, r) in &storage {
            let b = ((rt / bin_w) as usize).min(n_bins - 1);
            match &mut bins[b] {
                Some((_, hull)) => hyperrectangle_grow_to_convex_hull(hull, r),
                None => bins[b] = Some((*rt, *r)),
            }
        }
        for (bt, r) in bins.iter().flatten() {
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
        if !safe {
            n_unsafe += 1;
        }
        wall_sum_us += wall_us;
        if wall_us > wall_max_us {
            wall_max_us = wall_us;
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
