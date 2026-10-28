import math

import pytest
from robot_nav_sim.detour import (detour_goal, needs_detour,
                                  obstacle_in_robot_frame, robot_to_world)

FOV = 1.047  # The simulated camera's horizontal field of view.


def line_distance(a, b, p):
    """Distance from p to the infinite line through a and b."""
    ab = (b[0] - a[0], b[1] - a[1])
    ap = (p[0] - a[0], p[1] - a[1])
    return abs(ab[0] * ap[1] - ab[1] * ap[0]) / math.hypot(*ab)


def is_left(a, b, p):
    """Return True if p is left of the direction from a to b."""
    return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) > 0


# --- obstacle_in_robot_frame ---

def test_centred_bbox_is_straight_ahead_past_the_camera():
    assert obstacle_in_robot_frame(160, 320, FOV, 2.0, 0.55) == \
        pytest.approx((2.55, 0.0))


def test_bbox_at_image_edge_is_at_half_the_field_of_view():
    # Column 0 is the left edge: lateral = depth * tan(fov / 2), to the left.
    x, y = obstacle_in_robot_frame(0, 320, FOV, 2.0, 0.0)
    assert (x, y) == pytest.approx((2.0, 2.0 * math.tan(FOV / 2)))
    x, y = obstacle_in_robot_frame(320, 320, FOV, 2.0, 0.0)
    assert y == pytest.approx(-2.0 * math.tan(FOV / 2))


# --- robot_to_world ---

def test_robot_to_world_translates_and_rotates():
    assert robot_to_world((0, 0, 0), (1, 2)) == pytest.approx((1, 2))
    # Facing +y: the robot's forward is world +y, its left is world -x.
    assert robot_to_world((3, 4, math.pi / 2), (1, 0)) == pytest.approx((3, 5))
    assert robot_to_world((3, 4, math.pi / 2), (0, 1)) == pytest.approx((2, 4))


# --- needs_detour ---

def test_obstacle_on_the_path_needs_a_detour():
    assert needs_detour((0, 0), (5, 0), (2.5, 0.0), 1.5)


def test_obstacle_with_room_beside_the_path_does_not():
    assert needs_detour((0, 0), (5, 0), (2.5, 1.2), 1.5)  # The sim's person.
    assert not needs_detour((0, 0), (5, 0), (2.5, 1.6), 1.5)


def test_obstacle_behind_the_robot_does_not():
    assert not needs_detour((3, 0), (5, 0), (2.5, 0.0), 1.5)


def test_obstacle_beyond_the_destination_uses_distance_to_destination():
    assert needs_detour((0, 0), (5, 0), (6.0, 0.0), 1.5)
    assert not needs_detour((0, 0), (5, 0), (7.0, 0.0), 1.5)


# --- detour_goal ---

def test_obstacle_left_of_path_passes_on_the_right():
    robot, obstacle, dest = (0.8, 0.0), (2.5, 1.2), (5.0, 0.0)
    goal = detour_goal(robot, obstacle, dest, 1.5, 0.5)
    assert is_left(robot, goal, obstacle)  # Driving to goal, obstacle on our left.
    assert line_distance(robot, goal, obstacle) == pytest.approx(1.5)


def test_obstacle_right_of_path_passes_on_the_left():
    robot, obstacle, dest = (0.8, 0.0), (2.5, -1.2), (5.0, 0.0)
    goal = detour_goal(robot, obstacle, dest, 1.5, 0.5)
    assert not is_left(robot, goal, obstacle)
    assert line_distance(robot, goal, obstacle) == pytest.approx(1.5)


def test_obstacle_dead_ahead_passes_on_the_left():
    robot, obstacle = (0.0, 0.0), (2.0, 0.0)
    goal = detour_goal(robot, obstacle, (5.0, 0.0), 1.5, 0.5)
    assert goal[1] > 0
    assert line_distance(robot, goal, obstacle) == pytest.approx(1.5)


def test_goal_is_past_distance_beyond_the_closest_approach():
    # Obstacle 2.5 m away, pass at 1.5 m: the tangent is 2.0 m long.
    goal = detour_goal((0, 0), (2.5, 0.0), (5, 0), 1.5, 0.5)
    assert math.hypot(*goal) == pytest.approx(2.0 + 0.5)


def test_too_close_backs_off_sideways():
    goal = detour_goal((0, 0), (1.0, 0.0), (5, 0), 1.5, 0.5)
    assert goal == pytest.approx((0.0, 0.5))
