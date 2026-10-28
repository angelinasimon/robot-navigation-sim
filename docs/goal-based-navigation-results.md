# Goal-based navigation: before vs after (#12)

## What changed

| | Before (`robot_driver.py`) | After (`driver_node` + planner goals) |
|---|---|---|
| Who decides where to go | Driver: fixed goal, plus a reactive BRAKE → REVERSE → TURN → CREEP routine when the planner says AVOID_LEFT/RIGHT | Planner: publishes the active goal on `/goal`, the destination or a temporary detour goal beside a blocking obstacle |
| Who drives | Python goal-seeking inside the same node | C++ `Controller` via `DriverLogic`, unit-tested without ROS |
| Depth camera | Triggers the avoidance routine; stale depth ignored (drives blind) | Safety stop only; stale or missing depth stops forward motion (`DEPTH_STALE`) |
| Planner → driver | `cmd` strings | `/goal` (PoseStamped) and `/driver/status` back, echoing the goal it refers to |

## Method

- **Scenario:** `off_center_person` in `worlds/reactive_avoidance_demo.sdf`. The robot starts at (0, 0) facing +x with goal (5, 0). A standing person is at (2.5, 1.2): 1.2 m left of the straight path, so passing them needs some deviation.
- **Runs:** 3 per version, with avoidance on (`./scripts/run_reactive_demo.sh`), logged by `eval_logger_node` to `data/eval_trials.csv`:
  ```
  ros2 run robot_nav_sim eval_logger_node --ros-args -p trial_id:=<id> \
    -p scenario_type:=off_center_person -p obstacle_x:=2.5 -p obstacle_y:=1.2 -p timeout_s:=120.0
  ```
  The baseline (`baseline_001`–`003`) was run on `main`, whose simulation code (`src/robot_nav_sim`, `scripts`, `worlds`) matched `PID-driving` before #12: `git diff` shows only the then-unused `DriverStatus` message and `__pycache__` files. The after runs (`after_001`–`003`) were on `feat/12-planner-goals`. One baseline attempt is marked INVALID in the CSV (the sim was left paused) and is excluded.
- **Prediction, written before the after runs:** faster, and a *closer* pass than the baseline, around 1.5–1.6 m, because the detour is designed to pass the person's centre at `pass_distance` = 1.5 m.

## Results

| Metric | Before | After | Change |
|---|---|---|---|
| Success (reached goal, no timeout) | 3/3 | 3/3 | same |
| Time to goal, mean (runs) | **33.0 s** (35.7, 33.1, 30.3) | **20.6 s** (20.5, 21.3, 20.1) | **−12.4 s (−38%)** |
| Time to goal, spread (sd) | 2.7 s | 0.6 s | 4.5× more consistent |
| Closest approach to the person (centre to centre), mean | 1.96 m (1.92, 1.96, 1.99) | 1.54 m (1.54, 1.54, 1.54) | −0.42 m |
| Driver safety stops (`SAFETY_STOP`) | n/a | 0 in all runs | |

In every after run the planner logged one detour and no timeouts, e.g.:
```
[DETOURING]  Obstacle at (2.40, 0.99) blocks the path: detouring via (2.55, -0.58)
[NAVIGATING] Detour goal reached: resuming to destination (5.00, 0.00)
[ARRIVED]    Destination reached
```

## Interpretation

- **Faster and more repeatable.** The old driver stopped, reversed, turned and crept past the person, and how long that took varied from run to run. The new robot keeps moving: one smooth detour, then straight to the goal.
- **Closer, by design, and as predicted.** The robot now passes 1.54 m from the person's centre, where the old one swerved out to about 1.96 m. That is `pass_distance` (1.5 m) plus a little. Using the vehicle's footprint parameters (half-width 0.55 m), that leaves roughly 0.7 m between the robot's side and the person. If more clearance is wanted, `pass_distance` is now a single, named knob.
- **The position estimate errs on the safe side.** The planner placed the person at y ≈ 0.99 m, against the true 1.2 m: YOLO only sees the part of the person inside the image edge, so the box centre sits closer to the path than the person's centre. A closer obstacle means a bigger detour.

## Limitations

- **3 runs per version, one scenario.** The difference in time (12 s, against a spread of 3 s or less) is large enough to trust. The other named scenarios in `docs/evaluation.md` and randomized trials are not covered yet.
- **Time includes a human step.** `eval_logger_node` starts its clock when it starts, and the operator then presses Play, so a second or two of each time is reaction time. That inflates both versions about equally, but it adds noise.
- **Collisions are not measured automatically.** The CSV's `collision` column is empty. No contact was seen, and the closest approach leaves clear space, but this rests on observation.
- **Untested here:** an obstacle dead ahead (where the robot must pass much closer to its own path), a second obstacle appearing during a detour (detections are ignored while detouring), and turning in place close to an obstacle after a `SAFETY_STOP`.
