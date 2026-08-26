# quad_reach_monitor — RusTReach runtime monitor as a cp310 wheel

Python bindings (pyo3 0.22, maturin) around `exps/quadcopter_monitor`'s core:
anytime face-lifting reach sets of the 12-D DJI-F450 model, checked in 3-D
against the ANSR city elevation grid, per flight segment.

## Build (inside `ansr_vandy:rust`, the `ebt/rust_thetastar` pattern)

The wheel must target the container's Python 3.10 (host is 3.12 — do NOT build
on the host for container use):

```bash
docker run --rm \
  -v /home/johnsott/ansr-final/github/rustreach:/rustreach \
  -v <out>:/wtarget -v $HOME/.cargo/registry:/root/.cargo/registry \
  --entrypoint bash ansr_vandy:rust -c \
  "source /root/.cargo/env && cd /rustreach/exps/quad_reach_monitor_py && \
   CARGO_TARGET_DIR=/wtarget maturin build --release --interpreter python3.10 \
   --out /wtarget/wheels"
```

Produces `quad_reach_monitor-0.1.0-cp310-cp310-manylinux_*_x86_64.whl`
(cf. `theta_star_bi-0.1.0-cp310-...whl` shipped in `release_installer/docker/`).

## Python API

```python
import quad_reach_monitor as qrm

qrm.init_city("city_elev.f32", 1000, 1000, 1.0, -500.0, -500.0,
              0.5,   # lateral clearance margin [m] (added to 0.16 m quad half-width)
              1.0)   # vertical clearance margin [m]

# state12 = [x_north, y_east, z_alt_up, phi, theta, psi, u, v, w, p, q, r]
safe, wall_us, iters, n_boxes, hull = qrm.check_segment(
    state12, [gx, gy, gz],  # goal (next waypoint), same frame
    2.0,    # reach horizon [s]
    100,    # anytime wall-clock budget [ms]
    0.0,    # pos_unc: initial-set position uncertainty half-width [m]
    0.0)    # vel_unc: velocity uncertainty half-width [m/s]

# or with the (0.05 s bin-hull) reach boxes for display/logging:
safe, wall_us, iters, boxes = qrm.check_segment_boxes(...)
```

`check_segment` releases the GIL during the reach computation.

## ROS 2 live-veto wiring (documented, NOT wired — per task instructions)

Node `reach_monitor_node` (python, runs in the ansr_vandy container where the
wheel is pip-installed, exactly like `theta_star_bi`):

1. **State in**: subscribe `/adk_node/SimpleFlight/odom_local_ned_10hz`
   (`nav_msgs/Odometry`; the ebt stack subscribes the airsim alias of the same
   topic at `ebt/ebt_node.py:36`, `ebt/bt/composite_construction/topics_processing.py:122`).
   Convert NED -> monitor frame: `x=pos.x (N), y=pos.y (E), z=-pos.z (alt-up)`,
   `u=twist.x, v=twist.y, w=-twist.z` (bag replays lack twist; use finite
   differences as in the offline demo).
2. **Plan in**: subscribe the planned `adk_node.msg.WaypointPath` on
   `adk_node/input/waypoints` — or better, interpose BEFORE publication: the
   ebt node publishes it at `ebt/ebt_node.py:55`
   (`create_publisher(adk_node.msg.WaypointPath, 'adk_node/input/waypoints', 1)`;
   also `ebt/bt/composite_construction/publishing_processing.py:70`). Rename the
   ebt output topic to `.../waypoints_proposed` and let the monitor own the real one.
3. **Check**: for each consecutive waypoint pair along the proposed path within
   the next horizon, call `qrm.check_segment(state12, wp, horizon, budget_ms,
   pos_unc, vel_unc)`; budget_ms = 100 / (#segments checked) to keep the whole
   veto cycle inside one 10 Hz tick (the anytime property makes the per-call
   budget a hard knob).
4. **Veto**: if any segment flags, do NOT forward the `WaypointPath`; publish a
   hold (current position as single waypoint / previous safe prefix) and a
   `std_msgs/String` diagnostic on `/vanderbilt/reach_monitor/verdict`.
   This gates the same actuation path the BT's `check_not_in_obstacle`
   guard uses, but with a dynamics-aware reach set instead of a point lookup.
5. **Grid in**: `init_city` from the packaged `city_1000_1000.npz` (exported to
   `.f32`) or subscribe `/vanderbilt/obstacle_map/data` (raw float heights,
   `world_model/obstacle_map_node2.py`) and rebuild the grid on change.

Measured on the recorded scenario-007 flight (offline replay, 24-core host):
~86 ms avg / 204 steps at a 100 ms budget, 11-12 anytime iterations, ~41k reach
boxes checked per step; see
`/home/johnsott/ansr-final/artifacts/sim/STATUS-rustreach-monitor.md`.
