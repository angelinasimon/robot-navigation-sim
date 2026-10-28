from robot_nav_sim.detour import detour_goal
from robot_nav_sim.goal_planner import GoalPlanner, Mode, PlannerParams

DEST = (5.0, 0.0)
ROBOT = (0.8, 0.0)
PERSON = (2.5, 1.2)    # Blocks the path: 1.2 m from it, under pass_distance.
FAR_SIDE = (2.5, 3.0)  # Plenty of room: no detour needed.


def detouring_planner(**overrides):
    planner = GoalPlanner(PlannerParams(destination=DEST, **overrides))
    planner.on_obstacle(ROBOT, PERSON, now=0.0)
    return planner


def test_starts_navigating_to_the_destination():
    planner = GoalPlanner(PlannerParams(destination=DEST))
    assert planner.mode == Mode.NAVIGATING
    assert planner.goal == DEST


def test_blocking_obstacle_switches_to_a_detour_goal():
    planner = GoalPlanner(PlannerParams(destination=DEST))
    assert planner.on_obstacle(ROBOT, PERSON, now=0.0) is not None
    assert planner.mode == Mode.DETOURING
    assert planner.goal == detour_goal(ROBOT, PERSON, DEST, 1.5, 0.5)


def test_obstacle_with_room_to_pass_is_ignored():
    planner = GoalPlanner(PlannerParams(destination=DEST))
    assert planner.on_obstacle(ROBOT, FAR_SIDE, now=0.0) is None
    assert planner.mode == Mode.NAVIGATING and planner.goal == DEST


def test_avoidance_disabled_never_detours():
    planner = detouring_planner(avoidance_enabled=False)
    assert planner.mode == Mode.NAVIGATING and planner.goal == DEST


def test_detections_while_detouring_do_not_move_the_goal():
    planner = detouring_planner()
    goal = planner.goal
    assert planner.on_obstacle((1.5, -0.2), (2.6, 1.0), now=1.0) is None
    assert planner.goal == goal


def test_reaching_the_detour_goal_resumes_to_the_destination():
    planner = detouring_planner()
    assert planner.on_goal_reached(planner.goal) is not None
    assert planner.mode == Mode.NAVIGATING and planner.goal == DEST


def test_stale_report_about_the_old_goal_is_ignored():
    # The race the goal echo in DriverStatus exists for: just after the
    # planner switches back to the destination, the driver still reports
    # GOAL_REACHED for the detour goal. That must not count as arriving.
    planner = detouring_planner()
    detour = planner.goal
    planner.on_goal_reached(detour)
    assert planner.on_goal_reached(detour) is None
    assert planner.mode == Mode.NAVIGATING


def test_reaching_the_destination_arrives_and_stays():
    planner = GoalPlanner(PlannerParams(destination=DEST))
    assert planner.on_goal_reached(DEST) == 'Destination reached'
    assert planner.mode == Mode.ARRIVED
    assert planner.on_obstacle((4.0, 0.0), (4.5, 0.0), now=1.0) is None
    assert planner.mode == Mode.ARRIVED and planner.goal == DEST


def test_detour_times_out_back_to_the_destination():
    planner = detouring_planner(detour_timeout=20.0)
    assert planner.on_tick(now=19.9) is None
    assert planner.mode == Mode.DETOURING
    assert planner.on_tick(now=20.0) is not None
    assert planner.mode == Mode.NAVIGATING and planner.goal == DEST


def test_timeout_does_nothing_unless_detouring():
    planner = GoalPlanner(PlannerParams(destination=DEST))
    assert planner.on_tick(now=1000.0) is None
    assert planner.mode == Mode.NAVIGATING
