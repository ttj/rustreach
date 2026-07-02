// Core of the RusTReach runtime monitor for the Vanderbilt ANSR stack:
// anytime reach sets of the 12-D quadcopter model checked in 3-D against a
// city elevation grid. Used by the replay bin (main.rs) and by the
// quad_reach_monitor_py maturin wheel (exps/quad_reach_monitor_py).
//
// Frame: x = north [m], y = east [m], z = altitude UP [m] (NED with z negated).

use std::sync::Mutex;
use std::time::Instant;

use lazy_static::lazy_static;

use rtreach::face_lift::{face_lifting_iterative_improvement, LiftingSettings, ITERATIONS_AT_QUIT};
use rtreach::geometry::{hyperrectangle_grow_to_convex_hull, HyperRectangle};

use quadcopter::controller::model_sample_action;
use quadcopter::dynamics_quadcopter::{QuadcopterModel, QUAD_NUM_DIMS};

pub const NUM_DIMS: usize = QUAD_NUM_DIMS;
pub const QUAD_HALF_WIDTH: f64 = 0.16; // m, same bloat as quadcopter_model.rs

pub struct City {
    pub nx: usize,
    pub ny: usize,
    pub res: f64,
    pub ox: f64,
    pub oy: f64,
    pub elev: Vec<f32>, // row-major [ix][iy]
    pub lat_margin: f64,
    pub vert_margin: f64,
}

lazy_static! {
    pub static ref CITY: Mutex<Option<City>> = Mutex::new(None);
}

pub fn set_city(city: City) {
    *CITY.lock().unwrap() = Some(city);
}

/// 3-D check of one reach box against the city elevation grid.
/// Returns true if SAFE.
pub fn city_box_safe(r: &HyperRectangle<NUM_DIMS>) -> bool {
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

pub struct SegmentResult {
    pub safe: bool,
    pub wall_us: f64,
    pub iterations: u64,
    /// (time, reach box) pairs from the finest completed anytime iteration.
    pub storage: Vec<(f64, HyperRectangle<NUM_DIMS>)>,
}

/// Run one monitor step: anytime reach set from `state` (12-D: x,y,z,
/// phi,theta,psi, u,v,w, p,q,r) under the goal-conditioned controller headed
/// at `goal`, over `horizon` seconds within `wall_ms` milliseconds, with the
/// initial set inflated by `pos_unc` (m) / `vel_unc` (m/s), checked against
/// the city grid installed via `set_city`.
pub fn check_segment(
    state: [f64; NUM_DIMS],
    goal: [f64; 3],
    horizon: f64,
    wall_ms: u64,
    pos_unc: f64,
    vel_unc: f64,
) -> SegmentResult {
    let mut quad = QuadcopterModel::default();
    quad.set_ctrl_fn(model_sample_action);
    quad.set_goal(goal);
    let ctrl_input = quad.sample_state_action(&state).to_vec();

    let mut set: LiftingSettings<NUM_DIMS> = LiftingSettings::<NUM_DIMS> {
        init: HyperRectangle::default(),
        reach_time: horizon,
        initial_step_size: 0.1,
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
        && state[6].abs() < 1e-9
        && state[7].abs() < 1e-9
        && state[8].abs() < 1e-9
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
            true,
            &mut storage,
            false,
            true,
        );
        (s, *ITERATIONS_AT_QUIT.lock().unwrap())
    };
    let wall_us = t_start.elapsed().as_micros() as f64;

    SegmentResult { safe, wall_us, iterations, storage }
}

/// Compress a stored reach tube to per-time-bin convex hulls (display only).
pub fn bin_hulls(
    storage: &[(f64, HyperRectangle<NUM_DIMS>)],
    horizon: f64,
    bin_w: f64,
) -> Vec<(f64, HyperRectangle<NUM_DIMS>)> {
    let n_bins = (horizon / bin_w).ceil() as usize + 1;
    let mut bins: Vec<Option<(f64, HyperRectangle<NUM_DIMS>)>> = vec![None; n_bins];
    for (rt, r) in storage {
        let b = ((rt / bin_w) as usize).min(n_bins - 1);
        match &mut bins[b] {
            Some((_, hull)) => hyperrectangle_grow_to_convex_hull(hull, r),
            None => bins[b] = Some((*rt, *r)),
        }
    }
    bins.into_iter().flatten().collect()
}
