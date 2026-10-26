# Controller Study: Heading Wraparound Near ±π

Issue: #4. Result: **no material impact; no change to the controller.**

## Question

The controller's heading error `alpha` is wrapped to the range (−π, π]. When the goal is almost directly behind the robot, a small heading error can move `alpha` from about +π to about −π. Angular velocity is `ka · alpha`, clamped to ±1.5 rad/s, so that wrap would reverse the commanded turn direction.

Does heading noise make the robot dither (turn left, right, left, ...) instead of committing to a turn? If so, does it delay navigation enough to matter?

## Method

The study is `src/robot_nav_controller/experiments/wraparound_study.cpp`. It uses the same unicycle physics (`test/unicycle.hpp`) and the same 50 Hz step as the closed-loop test.

- **Setup:** the robot starts at the origin facing +x, with the goal 2 m away, directly behind it or offset by ±0.01 or ±0.05 rad. Default controller gains are used.
- **Noise:** Gaussian noise with standard deviation σ ∈ {0, 0.01, 0.02, 0.05, 0.1} rad is added to the heading the controller *sees*. This is measurement noise: the true pose moves with noise-free physics.
- **Trials:** 200 per configuration. Each trial uses a fixed seed, and the same seeds are reused for every configuration. That means rows differ only in the configuration, and reruns give identical output on the same compiler and standard library.
- **Metrics:**
  - **reversals:** how many times the commanded turn direction flips before the robot commits to a turn
  - **commit time:** seconds until the true heading error is under π/2. Even with no noise this is about 1.06 s, the time needed to turn 90° at 1.5 rad/s. Dithering shows up as time *above* that.
  - **time to goal**, and how many trials reach the goal.

## Predictions (recorded before running)

- **Mechanism:** the robot turns about 0.03 rad (1.7°) per step, which is smaller than the noise at σ = 0.05. So the direction near the tie is close to random until the robot has turned past roughly 2σ.
- **Prediction A:** negligible at σ ≤ 0.02; about 0.2 s of dithering at σ = 0.05; about 1 s at σ = 0.1.
- **Prediction B:** dithering for 1–2 s at σ = 0.05, possibly self-reinforcing ("oscillating leads to more oscillating").

## Decision rule (set before running)

At σ = 0.05 rad, mitigate if **either** of these holds:
- the 95th-percentile commit time is more than **0.5 s** above the noise-free value, or
- the median number of reversals is **more than 3**.

σ = 0.05 rad (about 3°) was chosen as a plausible level of yaw-estimate noise. It has not been measured on this robot yet.

## Results

```
 offset  sigma | reversals       | commit time (s)       | goal (s) | reached
               |  med  p95   max |    med    p95     max |      med |
   0.00   0.00 |    0    0     0 |   1.06   1.06    1.06 |     7.38 | 200/200
   0.00   0.01 |    0    0     0 |   1.06   1.06    1.06 |     7.38 | 200/200
   0.00   0.02 |    0    2     2 |   1.06   1.10    1.14 |     7.38 | 200/200
   0.00   0.05 |    0    4     8 |   1.06   1.14    1.26 |     7.44 | 200/200
   0.00   0.10 |    2    7    12 |   1.10   1.30    1.50 |     7.66 | 200/200

   0.01   0.00 |    0    0     0 |   1.06   1.06    1.06 |     7.36 | 200/200
   0.01   0.01 |    0    0     0 |   1.06   1.06    1.06 |     7.36 | 200/200
   0.01   0.02 |    0    1     2 |   1.06   1.10    1.10 |     7.38 | 200/200
   0.01   0.05 |    0    4     9 |   1.06   1.14    1.34 |     7.44 | 200/200
   0.01   0.10 |    2    7    16 |   1.10   1.30    1.54 |     7.64 | 200/200

  -0.01   0.00 |    0    0     0 |   1.06   1.06    1.06 |     7.36 | 200/200
  -0.01   0.01 |    0    0     1 |   1.06   1.06    1.10 |     7.36 | 200/200
  -0.01   0.02 |    0    1     2 |   1.06   1.10    1.10 |     7.38 | 200/200
  -0.01   0.05 |    0    3     6 |   1.06   1.14    1.22 |     7.44 | 200/200
  -0.01   0.10 |    2    7    12 |   1.10   1.30    1.50 |     7.66 | 200/200

   0.05   0.00 |    0    0     0 |   1.02   1.02    1.02 |     7.34 | 200/200
   0.05   0.01 |    0    0     0 |   1.02   1.02    1.02 |     7.34 | 200/200
   0.05   0.02 |    0    0     1 |   1.02   1.02    1.10 |     7.34 | 200/200
   0.05   0.05 |    0    2     6 |   1.02   1.10    1.22 |     7.40 | 200/200
   0.05   0.10 |    2    6    11 |   1.06   1.26    1.42 |     7.60 | 200/200

  -0.05   0.00 |    0    0     0 |   1.02   1.02    1.02 |     7.34 | 200/200
  -0.05   0.01 |    0    0     0 |   1.02   1.02    1.02 |     7.34 | 200/200
  -0.05   0.02 |    0    0     0 |   1.02   1.02    1.02 |     7.34 | 200/200
  -0.05   0.05 |    0    2     6 |   1.02   1.14    1.22 |     7.40 | 200/200
  -0.05   0.10 |    2    7     8 |   1.06   1.22    1.38 |     7.60 | 200/200
```

**Observations:**
- **Without noise there are no reversals.** The (−π, π] wrap breaks an exact tie toward +π (turn left), and once the robot is turning, the error moves away from the tie. Unit tests cover this case.
- **With noise, reversals do happen**, but each lasts about one 20 ms step. Even the worst trial at σ = 0.1 loses under 0.5 s.
- **The dithering does not feed on itself.** Each step that turns the right way moves the goal away from directly behind, which makes the next flip less likely. Doubling the noise from 0.05 to 0.1 raises the worst-case commit time only from 1.26 s to 1.50 s, and every trial commits and reaches the goal.

## Decision

At σ = 0.05:
- the 95th-percentile commit time is 1.14 s against 1.06 s with no noise, which is **+0.08 s**, under the 0.5 s limit
- the median number of reversals is **0**, under the limit of 3

The rule does not trigger, so **the controller is unchanged.**

Comparing with the predictions:
- **A** was about right on size but described the worst case; a typical trial loses nothing.
- **B** overestimated the duration, and the data does not support self-reinforcement.

## When to revisit

- Real yaw noise turns out to be well above about 0.1 rad.
- Turn rate is reduced: lower `ka` or `max_angular_velocity` means smaller turns per step, so the robot takes longer to escape the tie.
- The control rate drops well below 50 Hz.
- Navigation needs a guaranteed single turn direction.

In any of these cases, rerun the study with the new settings. A stateless mitigation (a fixed turn direction when |alpha| > π − δ) was considered and not needed.

## How to rerun

```sh
cmake -S src/robot_nav_controller -B /tmp/robot-nav-controller \
  -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON
cmake --build /tmp/robot-nav-controller
/tmp/robot-nav-controller/wraparound_study
```
