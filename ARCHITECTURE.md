# Architecture

Source map checked at `7b264deba401bca0bf6131ae02507b324640864e`.
This TTJ fork of RusTReach preserves the upstream Rust reachability research implementation and adds a Vanderbilt quadcopter replay monitor with Python bindings. The common algorithm computes finite-horizon reach-set overapproximations using face lifting. Verdicts concern the selected mathematical model, initial uncertainty and supplied obstacles; they do not establish that the physical vehicle, perception or map satisfies those assumptions.

## Read first

- [README.md](README.md) describes the upstream research method, citations, bicycle/quadcopter experiments and output recipes.
- [Cargo.toml](Cargo.toml) lists the Rust workspace members.
- [exps/quad_reach_monitor_py/README.md](exps/quad_reach_monitor_py/README.md) documents TTJ's monitor wheel, frame/API and a proposed ROS integration. Its live-veto section explicitly says it is documented, not wired; its measured timing is a dated offline replay result.
- The existing lab [surrogate/reach.py](https://github.com/ttj/vu-autonomy/blob/main/surrogate/reach.py) explains how the fork is consumed, including the distinction between the surrogate's primary 6-D point model and this fork's 12-D quadcopter monitor.

## Module map

| Area | Responsibility and useful seam |
| --- | --- |
| [rtreach/](rtreach) | Common library. `face_lift.rs` provides `face_lifting_iterative_improvement` and `LiftingSettings`; `geometry.rs` / `interval.rs` represent sets; `system_model.rs:SystemModel` supplies derivative/control callbacks; `obstacle_safety.rs` holds generic obstacle checks. |
| [bicycle/](bicycle), [quadcopter/](quadcopter) | Vehicle dynamics, controllers, simulation and utilities. The quadcopter model is the linearized 12-D DJI F450 model described in `dynamics_quadcopter.rs`, not a measured Tello/Neo2 plant model. |
| [exps/](exps) | Upstream corridor/neighborhood/simple-control experiments and path-generation programs. They combine dynamics, policy/model inputs, obstacles and output generation; each package has its own entrypoint. |
| [models/](models), [eval_input_data/](eval_input_data) | Existing learned policy weights and experiment inputs. Their presence does not turn every monitor call into an RL-policy evaluation. |
| [exps/quadcopter_monitor/src/lib.rs](exps/quadcopter_monitor/src/lib.rs) | TTJ shared core: `City`, `set_city()`, `city_box_safe()`, `check_segment()` and `SegmentResult`. Checks reach boxes against a 3-D elevation-grid clearance condition. `bin_hulls()` compresses stored boxes for display. |
| [exps/quadcopter_monitor/src/main.rs](exps/quadcopter_monitor/src/main.rs) | Offline replay CLI: reads elevation floats and flight-segment CSV, invokes the shared monitor and writes verdict/timing/box CSVs. |
| [exps/quad_reach_monitor_py/](exps/quad_reach_monitor_py) | Standalone PyO3/maturin crate, separate from the root workspace. Exports `init_city()`, `check_segment()` and `check_segment_boxes()` as Python module `quad_reach_monitor`. |
| [bash_scripts/](bash_scripts), [scripts/](scripts) | Existing experiment/analysis orchestration; inspect individual scripts and output paths before running bulk experiments. |

## Reach-set and monitor flow

The common face-lifting algorithm receives a `SystemModel`, an initial hyperrectangle, horizon/runtime settings and callbacks. The anytime procedure refines reach-set overapproximation by reducing the step size while its termination/budget conditions permit. Research experiments choose vehicle dynamics, controller/policy and obstacle behavior; their settings belong to the experiment, not to a universal lab safety certificate.

The TTJ monitor installs a `City` grid through `set_city()` (Rust) or `init_city()` (Python). The Python loader requires exactly `nx * ny * 4` bytes of little-endian float32 elevation data. The grid is row-major `[ix][iy]`, with north-index `ix` and east-index `iy`; origin/resolution/margins accompany the values. The monitor frame is `x=north`, `y=east`, `z=altitude up`. NED data therefore negates its down coordinate; the lab's ENU adapter instead swaps east/north and keeps altitude up.

`check_segment()` takes `[x,y,z,phi,theta,psi,u,v,w,p,q,r]`, a 3-D goal, horizon, requested wall-clock budget and position/velocity uncertainty half-widths. It creates the quadcopter model with its existing goal-conditioned controller, inflates the initial position/velocity intervals and runs face lifting with city checks. This monitor constructs the default model without loading an ONNX actor; research experiments can use learned policies separately. The returned `safe`, measured `wall_us`, iteration count and stored reach boxes describe that modeled calculation.

`city_box_safe()` expands horizontal bounds by quad half-width plus lateral margin and compares elevation with the reach box's lower altitude minus vertical margin. The process-global `CITY` is protected by a mutex; the Python functions release the GIL during reach computation. Callers must initialize the intended grid before checking: when no city is installed, the current city callback returns `true` rather than rejecting an uninitialized environment. `bin_hulls()`'s per-time-bin hulls are for display; the monitor verdict uses the underlying reach checks.

## Build and entrypoints

Use the existing README's `cargo run --release -p <experiment>` pattern for a selected upstream experiment, after checking its arguments and inputs. The TTJ replay entrypoint is workspace package `quadcopter_monitor`; its `main.rs` usage lists the elevation path/dimensions/resolution/origin, input CSV, output directory and optional budget/horizon/margin/uncertainty values. It creates or appends output files according to `start_idx`; choose a separate output directory for a new replay.

For Python, use the existing [maturin pyproject](exps/quad_reach_monitor_py/pyproject.toml) and [standalone Cargo manifest](exps/quad_reach_monitor_py/Cargo.toml). The wheel README's Python 3.10 container recipe serves that container's ABI. The lab's `surrogate/reach.py` separately documents host Python 3.12 builds; choose the interpreter for the actual consumer rather than copying a wheel filename across environments. Building only the root workspace does not build this standalone wheel.

The lab already uses `surrogate.reach:RustReachChecker` / `build_elevation_grid` and the `CollisionGuard` backend seam. Its primary surrogate point-model binding lives in `vu-autonomy/rust/point_reach`, which path-depends on this repository's `rtreach` library; the optional `quad12` model consumes `quad_reach_monitor`. Reuse those adapters and tests rather than adding a second guard, rasterizer or coordinate conversion here. No Rust build, recorded-flight scan or live monitor was run for this documentation change.

## Traps and change boundaries

- Model identity matters: linearized F450 quad12, bicycle and surrogate point dynamics answer different reachability questions. A conservative or passing result under one is not interchangeable with another.
- `CITY` is shared across checks in a process, not owned by an individual Python object. Grid initialization, frame, resolution, uncertainty and margins are part of the caller's contract; module import alone supplies none of them.
- Requested budget and measured `wall_us` are different quantities. The README's historical replay timing does not prove a hard deadline on another host or input.
- Display hulls and plotted trajectories are compressed evidence, not the full predicate or proof that the operational plant follows the model.
- The wheel README's ROS topic/veto plan remains a proposed integration. Do not infer that a live publication gate is installed because bindings or replay CSVs exist.
- No GitHub Actions workflow is checked in at this source snapshot. Source/doc validation does not replace build/runtime evidence or model/physical qualification.

Index searched: lab CODE_MAP/CODE_INDEX, dependency/external-project notes, safe sibling references and existing reach/guard adapters; target README/workspace/model/face-lift/monitor/binding sources and current TTJ origin/all-state architecture PR/issue search.

Reusing: `rtreach::face_lift`, `SystemModel`, existing vehicle models, shared `quadcopter_monitor` core, PyO3 bindings and canonical `vu-autonomy` reach/guard/frame adapters. New because: this safe TTJ fork lacked a root architecture onboarding guide; no algorithm, guard, model or deployment is changed.
