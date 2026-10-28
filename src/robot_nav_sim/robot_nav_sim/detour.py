"""
Geometry for steering around an obstacle with a temporary goal.

Pure functions with no ROS dependency, so they can be unit-tested directly.

Frames: the robot frame has x forward and y to the left; the world frame is
the odometry frame. Points are (x, y) tuples, poses are (x, y, yaw) tuples,
angles are in radians and counter-clockwise positive.
"""

import math


def obstacle_in_robot_frame(bbox_center_x, image_width, horizontal_fov, depth,
                            camera_forward_offset):
    """
    Return the obstacle's (x, y) position in the robot frame.

    depth is the depth-image value at the obstacle: its distance along the
    camera's optical axis, not along the line of sight. For a pinhole camera,
    a pixel u columns from the image centre then sits depth * u / focal_length
    to the side, with pixels left of the centre on the robot's left (+y).
    The camera is assumed to face straight ahead, camera_forward_offset in
    front of the robot's origin.
    """
    focal_length = (image_width / 2.0) / math.tan(horizontal_fov / 2.0)
    lateral = depth * (image_width / 2.0 - bbox_center_x) / focal_length
    return (camera_forward_offset + depth, lateral)


def robot_to_world(robot_pose, point):
    """Convert a point from the robot frame to the world frame."""
    x, y, yaw = robot_pose
    px, py = point
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * px - s * py, y + s * px + c * py)


def _cross(a, b):
    """Z component of the 2-D cross product: > 0 when b is left of a."""
    return a[0] * b[1] - a[1] * b[0]


def needs_detour(robot_xy, destination, obstacle, pass_distance):
    """
    Return True if the obstacle blocks the straight path to destination.

    Blocking means ahead of the robot and closer than pass_distance to the
    segment from the robot to the destination. An obstacle beside the path
    with enough room, or one already behind the robot, needs no detour.
    """
    path = (destination[0] - robot_xy[0], destination[1] - robot_xy[1])
    to_obstacle = (obstacle[0] - robot_xy[0], obstacle[1] - robot_xy[1])
    path_length_sq = path[0] ** 2 + path[1] ** 2
    if path_length_sq == 0.0:
        return False
    # Position of the obstacle's closest point along the path: 0 at the robot,
    # 1 at the destination.
    t = (path[0] * to_obstacle[0] + path[1] * to_obstacle[1]) / path_length_sq
    if t <= 0.0:
        return False  # Behind the robot.
    t = min(t, 1.0)
    closest = (robot_xy[0] + t * path[0], robot_xy[1] + t * path[1])
    return math.dist(closest, obstacle) < pass_distance


def detour_goal(robot_xy, obstacle, destination, pass_distance, past_distance):
    """
    Return a temporary goal that takes the robot past the obstacle.

    The robot passes on the side away from the obstacle: if the obstacle is
    left of the straight path to destination, it passes on the right, and
    vice versa (dead ahead passes on the left). The goal lies on the line from
    the robot that just grazes a circle of radius pass_distance around the
    obstacle, past_distance beyond the point where that line is closest to the
    obstacle. So driving straight to the goal never brings the robot's centre
    nearer than pass_distance to the obstacle's centre.

    If the robot is already within pass_distance, no such line exists; the
    goal is then past_distance away at a right angle to the obstacle, on the
    same side, to back off sideways.
    """
    to_obstacle = (obstacle[0] - robot_xy[0], obstacle[1] - robot_xy[1])
    path = (destination[0] - robot_xy[0], destination[1] - robot_xy[1])
    # Obstacle left of the path (cross > 0): turn clockwise to pass right.
    side = -1.0 if _cross(path, to_obstacle) > 0.0 else 1.0

    distance = math.hypot(*to_obstacle)
    if distance <= pass_distance:
        offset, along = math.pi / 2.0, 0.0
    else:
        offset = math.asin(pass_distance / distance)
        along = math.sqrt(distance ** 2 - pass_distance ** 2)

    heading = math.atan2(to_obstacle[1], to_obstacle[0]) + side * offset
    length = along + past_distance
    return (robot_xy[0] + length * math.cos(heading),
            robot_xy[1] + length * math.sin(heading))
