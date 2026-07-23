# Evaluation Contract

## Project Goal

The goal of this project is to evaluate whether a simulated mobile robot can use camera-based perception to reach a goal without colliding with obstacles.

The detector is not evaluated in isolation. Detection quality matters because it affects the robot's ability to make safe navigation decisions.

## Evaluation Setup

The project will use two types of trials:

1. Named scenarios for debugging
2. Randomized trials for robustness

Named scenarios are repeatable test cases designed to isolate specific failure modes. Randomized trials test whether the system works beyond one hand-picked demo.

Initial evaluation plan:

- 5 named scenarios
- 20 randomized trials

## Named Scenarios

The initial named scenarios are:

1. Easy straight-line obstacle
2. Obstacle near start
3. Obstacle near goal
4. Narrow passage
5. Off-center obstacle

## System Metrics

### Success rate

Percentage of trials where the robot reaches the goal without colliding.

Why it matters: this is the main measure of whether the full autonomy loop works.

### Collision rate

Percentage of trials where the robot physically hits an obstacle.

Why it matters: a robot that reaches the goal but crashes is not successful.

### Timeout/stuck rate

Percentage of trials where the robot does not collide but also does not reach the goal within the time limit.

Why it matters: the robot should not avoid crashes by freezing forever.

### Time-to-goal

Time from trial start to reaching the goal.

Why it matters: this catches inefficient behavior, such as unnecessary stopping or very slow motion.

### Minimum distance to obstacle

Closest distance between the robot and an obstacle during the trial.

Why it matters: a trial can technically succeed while still passing dangerously close to an obstacle.

### Detection latency

Time between receiving a camera frame and publishing a detection or decision based on it.

Why it matters: even an accurate detector can fail the system if it reacts too late.

## Detector Metrics

### Recall

Percentage of real obstacles that the detector successfully detects.

Why it matters: recall is prioritized because a missed obstacle can cause a collision.

### Precision

Percentage of reported detections that are actually correct.

Why it matters: low precision can cause unnecessary stops or overly cautious behavior.

### IoU

Intersection over Union between the predicted bounding box and the ground-truth bounding box.

Why it matters: IoU measures whether the detector localized the obstacle accurately.

### mAP

Mean Average Precision across detection thresholds/classes.

Why it matters: mAP is useful later for comparing detector quality, but it is not the first priority because this project focuses on system behavior.

## Trial Log Format

Each trial should produce one CSV row.

```csv
trial_id,scenario_type,random_seed,start_x,start_y,start_yaw,goal_x,goal_y,obstacle_x,obstacle_y,obstacle_size_m,max_robot_speed_mps,detected_obstacle,first_detection_time_s,detection_latency_ms,min_distance_to_obstacle_m,success,collision,timeout,time_to_goal_s,notes
