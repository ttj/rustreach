// Python bindings for the RusTReach quadcopter runtime monitor
// (exps/quadcopter_monitor core), built as a cp310 wheel with maturin inside
// ansr_vandy:rust — same pattern as ebt/rust_thetastar.
//
// Python API:
//   import quad_reach_monitor as qrm
//   qrm.init_city(elev_f32_path, nx, ny, res, origin_x, origin_y,
//                 lat_margin, vert_margin)
//   safe, wall_us, iterations, n_boxes, hull = qrm.check_segment(
//       state12, goal3, horizon_s, wall_ms, pos_unc, vel_unc)
//   # state12: [x_north, y_east, z_alt_up, phi, theta, psi, u, v, w, p, q, r]
//   # hull: [xmin, xmax, ymin, ymax, zmin, zmax] over the reach tube
//   boxes = qrm.check_segment_boxes(...)  # + per-0.05 s-bin hull boxes

use pyo3::prelude::*;

use quadcopter_monitor::{bin_hulls, check_segment as core_check, set_city, City, NUM_DIMS};

#[pyfunction]
#[allow(clippy::too_many_arguments)]
fn init_city(
    elev_path: String,
    nx: usize,
    ny: usize,
    res: f64,
    origin_x: f64,
    origin_y: f64,
    lat_margin: f64,
    vert_margin: f64,
) -> PyResult<()> {
    let raw = std::fs::read(&elev_path)
        .map_err(|e| pyo3::exceptions::PyIOError::new_err(format!("{}: {}", elev_path, e)))?;
    if raw.len() != nx * ny * 4 {
        return Err(pyo3::exceptions::PyValueError::new_err(format!(
            "elev file size {} != nx*ny*4 = {}",
            raw.len(),
            nx * ny * 4
        )));
    }
    let mut elev = vec![0f32; nx * ny];
    for (k, ch) in raw.chunks_exact(4).enumerate() {
        elev[k] = f32::from_le_bytes([ch[0], ch[1], ch[2], ch[3]]);
    }
    set_city(City { nx, ny, res, ox: origin_x, oy: origin_y, elev, lat_margin, vert_margin });
    Ok(())
}

fn to_state(state: Vec<f64>) -> PyResult<[f64; NUM_DIMS]> {
    if state.len() != NUM_DIMS {
        return Err(pyo3::exceptions::PyValueError::new_err(format!(
            "state must have {} dims, got {}",
            NUM_DIMS,
            state.len()
        )));
    }
    let mut s = [0.0; NUM_DIMS];
    s.copy_from_slice(&state);
    Ok(s)
}

fn to_goal(goal: Vec<f64>) -> PyResult<[f64; 3]> {
    if goal.len() != 3 {
        return Err(pyo3::exceptions::PyValueError::new_err("goal must have 3 dims"));
    }
    Ok([goal[0], goal[1], goal[2]])
}

/// -> (safe, wall_us, iterations, n_boxes, [xmin,xmax,ymin,ymax,zmin,zmax])
#[pyfunction]
fn check_segment(
    py: Python<'_>,
    state: Vec<f64>,
    goal: Vec<f64>,
    horizon_s: f64,
    wall_ms: u64,
    pos_unc: f64,
    vel_unc: f64,
) -> PyResult<(bool, f64, u64, usize, Vec<f64>)> {
    let s = to_state(state)?;
    let g = to_goal(goal)?;
    let res = py.allow_threads(|| core_check(s, g, horizon_s, wall_ms, pos_unc, vel_unc));
    let mut hull = res.storage[0].1;
    for (_, r) in &res.storage {
        for d in 0..NUM_DIMS {
            hull.dims[d].min = hull.dims[d].min.min(r.dims[d].min);
            hull.dims[d].max = hull.dims[d].max.max(r.dims[d].max);
        }
    }
    Ok((
        res.safe,
        res.wall_us,
        res.iterations,
        res.storage.len(),
        vec![
            hull.dims[0].min, hull.dims[0].max,
            hull.dims[1].min, hull.dims[1].max,
            hull.dims[2].min, hull.dims[2].max,
        ],
    ))
}

/// -> (safe, wall_us, iterations, [[t,xmin,xmax,ymin,ymax,zmin,zmax], ...])
#[pyfunction]
fn check_segment_boxes(
    py: Python<'_>,
    state: Vec<f64>,
    goal: Vec<f64>,
    horizon_s: f64,
    wall_ms: u64,
    pos_unc: f64,
    vel_unc: f64,
) -> PyResult<(bool, f64, u64, Vec<Vec<f64>>)> {
    let s = to_state(state)?;
    let g = to_goal(goal)?;
    let res = py.allow_threads(|| core_check(s, g, horizon_s, wall_ms, pos_unc, vel_unc));
    let boxes = bin_hulls(&res.storage, horizon_s, 0.05)
        .into_iter()
        .map(|(t, r)| {
            vec![
                t,
                r.dims[0].min, r.dims[0].max,
                r.dims[1].min, r.dims[1].max,
                r.dims[2].min, r.dims[2].max,
            ]
        })
        .collect();
    Ok((res.safe, res.wall_us, res.iterations, boxes))
}

#[pymodule]
fn quad_reach_monitor(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(init_city, m)?)?;
    m.add_function(wrap_pyfunction!(check_segment, m)?)?;
    m.add_function(wrap_pyfunction!(check_segment_boxes, m)?)?;
    Ok(())
}
