"""
Decide which goal the robot drives to: the destination or a temporary detour.

No ROS dependency: planner_node feeds in events (an obstacle seen, the driver
reaching a goal, time passing) and publishes whatever goal this reports.
Each event method returns a message describing what changed, for the node to
log, or None if nothing changed.
"""

from dataclasses import dataclass
import enum
import math

from robot_nav_sim.detour import detour_goal, needs_detour


class Mode(enum.Enum):
    """What the planner is doing."""

    NAVIGATING = 'NAVIGATING'  # Driving to the destination.
    DETOURING = 'DETOURING'    # Driving to a temporary goal around an obstacle.
    ARRIVED = 'ARRIVED'        # Destination reached; nothing more to do.


@dataclass(frozen=True)
class PlannerParams:
    """Planner settings; distances in metres, times in seconds."""

    destination: tuple
    avoidance_enabled: bool = True
    pass_distance: float = 1.5   # Closest the robot's centre passes an obstacle's.
    past_distance: float = 0.5   # How far past the obstacle the detour goal is.
    detour_timeout: float = 20.0  # Give up on a detour goal after this long.


def _same_point(a, b):
    return math.isclose(a[0], b[0], abs_tol=1e-6) and math.isclose(a[1], b[1], abs_tol=1e-6)


class GoalPlanner:
    """State machine: NAVIGATING -> DETOURING -> NAVIGATING ... -> ARRIVED."""

    def __init__(self, params):
        self._params = params
        self.mode = Mode.NAVIGATING
        self.goal = params.destination  # The goal the driver should follow now.
        self._detour_start = None

    def on_obstacle(self, robot_xy, obstacle, now):
        """Start a detour if the obstacle blocks the path to the destination."""
        p = self._params
        if self.mode != Mode.NAVIGATING or not p.avoidance_enabled:
            # While detouring, new detections are ignored: replanning on every
            # frame would make the goal jump around and the robot weave.
            return None
        if not needs_detour(robot_xy, p.destination, obstacle, p.pass_distance):
            return None
        self.goal = detour_goal(robot_xy, obstacle, p.destination, p.pass_distance,
                                p.past_distance)
        self.mode = Mode.DETOURING
        self._detour_start = now
        return (f'Obstacle at ({obstacle[0]:.2f}, {obstacle[1]:.2f}) blocks the path: '
                f'detouring via ({self.goal[0]:.2f}, {self.goal[1]:.2f})')

    def on_goal_reached(self, reached_goal):
        """Handle the driver reporting GOAL_REACHED for reached_goal."""
        # Right after a goal switch the driver can still report the old goal.
        # Only a report about the current goal counts.
        if not _same_point(reached_goal, self.goal):
            return None
        if self.mode == Mode.DETOURING:
            return self._resume('Detour goal reached')
        if self.mode == Mode.NAVIGATING:
            self.mode = Mode.ARRIVED
            return 'Destination reached'
        return None

    def on_tick(self, now):
        """Abandon a detour that is taking too long (e.g. its goal is unreachable)."""
        if (self.mode == Mode.DETOURING
                and now - self._detour_start >= self._params.detour_timeout):
            return self._resume(f'Detour timed out after {self._params.detour_timeout:.0f} s')
        return None

    def _resume(self, reason):
        self.mode = Mode.NAVIGATING
        self.goal = self._params.destination
        self._detour_start = None
        return f'{reason}: resuming to destination ({self.goal[0]:.2f}, {self.goal[1]:.2f})'
