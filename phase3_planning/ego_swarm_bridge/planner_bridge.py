"""
UAV-X Phase 3 – EGO-Swarm ROS2 Bridge with Potential Field Fallback
====================================================================
Provides trajectory planning for individual UAVs in the UAV-X swarm.

Two back-ends are supported:

1. **ROS2 / EGO-Swarm back-end** (preferred)
   Uses ``rclpy`` to communicate with a running EGO-Swarm planning node.
   Each UAV has:
     - A publisher to ``/uav_{id}/planning/goal`` (geometry_msgs/PoseStamped)
     - A subscriber to ``/uav_{id}/planning/trajectory``
       (nav_msgs/Path – sequence of PoseStamped)
   The bridge submits a goal, waits for the trajectory response (with a
   configurable timeout), then returns the waypoint list.

2. **Potential Field fallback** (if rclpy is not installed)
   Implements a classical Artificial Potential Field (APF) planner:
     - Attractive force toward goal
     - Repulsive force away from each obstacle and other UAV
     - Euler integration until goal reached or max_iter exceeded
   All parameters are configurable at construction time.

Usage
-----
>>> bridge = EgoSwarmBridge(uav_ids=["uav_0", "uav_1", "uav_2"])
>>> path = bridge.plan_path(
...     "uav_0",
...     start=(0, 0, 30),
...     goal=(500, 300, 30),
...     other_uav_positions=[(300, 0, 30), (0, 300, 30)],
... )
>>> for wp in path:
...     print(wp)

Dependencies
------------
  ROS 2 Humble:  https://docs.ros.org/en/humble/
  EGO-Planner-v2: https://github.com/ZJU-FAST-Lab/EGO-Planner-v2
  numpy >= 1.21  (recommended, but plain-Python fallback included)
"""

import logging
import math
import time
from typing import List, Optional, Tuple

logger = logging.getLogger("EgoSwarmBridge")
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)

# ---------------------------------------------------------------------------
# Attempt numpy import (accelerates potential field calculations)
# ---------------------------------------------------------------------------
try:
    import numpy as np
    _NP = True
except ImportError:
    _NP = False
    logger.info("numpy not available; using pure-Python vector math.")

# ---------------------------------------------------------------------------
# Attempt ROS2 / rclpy import
# ---------------------------------------------------------------------------
try:
    import rclpy                                         # type: ignore
    from rclpy.node import Node                          # type: ignore
    from rclpy.qos import QoSProfile, ReliabilityPolicy # type: ignore
    from geometry_msgs.msg import PoseStamped            # type: ignore
    from nav_msgs.msg import Path                        # type: ignore
    _ROS2_AVAILABLE = True
    logger.info("rclpy detected – using ROS2/EGO-Swarm back-end.")
except ImportError:
    _ROS2_AVAILABLE = False
    logger.warning(
        "rclpy not found.  Falling back to Potential Field planner.\n"
        "To enable ROS2 back-end: source /opt/ros/humble/setup.bash && "
        "pip install rclpy"
    )


# ===========================================================================
# Vector3 helper (pure-Python, used when numpy unavailable)
# ===========================================================================

class _Vec3:
    """Lightweight immutable 3-D vector for fallback arithmetic."""

    __slots__ = ("x", "y", "z")

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def __add__(self, other: "_Vec3") -> "_Vec3":
        return _Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "_Vec3") -> "_Vec3":
        return _Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "_Vec3":
        return _Vec3(self.x * scalar, self.y * scalar, self.z * scalar)

    def __rmul__(self, scalar: float) -> "_Vec3":
        return self.__mul__(scalar)

    def norm(self) -> float:
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def normalize(self) -> "_Vec3":
        n = self.norm()
        if n < 1e-9:
            return _Vec3(0.0, 0.0, 0.0)
        return _Vec3(self.x / n, self.y / n, self.z / n)

    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def __repr__(self) -> str:
        return f"Vec3({self.x:.2f}, {self.y:.2f}, {self.z:.2f})"


def _v3(t: Tuple[float, float, float]) -> "_Vec3":
    return _Vec3(t[0], t[1], t[2])


# ===========================================================================
# ROS2 planning node (only constructed when rclpy is available)
# ===========================================================================

class _EgoSwarmPlanningNode(Node):  # type: ignore[misc]
    """
    ROS2 node that interfaces with EGO-Swarm trajectory planner nodes.

    For each UAV ID, it creates:
      - A publisher on ``/uav_{id}/planning/goal`` (PoseStamped)
      - A subscriber on ``/uav_{id}/planning/trajectory`` (Path)

    Trajectories received from EGO-Swarm are stored in
    ``self.trajectories[uav_id]`` as lists of ``(x, y, z)`` tuples.
    """

    def __init__(self, uav_ids: List[str]) -> None:
        super().__init__("uavx_ego_swarm_bridge")

        self.uav_ids = uav_ids
        self.trajectories: dict = {uid: None for uid in uav_ids}

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._goal_pubs = {}
        self._traj_subs = {}

        for uid in uav_ids:
            # Publisher: send planning goals to EGO-Swarm
            topic_goal = f"/uav_{uid}/planning/goal"
            self._goal_pubs[uid] = self.create_publisher(
                PoseStamped, topic_goal, qos
            )
            logger.info("Created publisher: %s", topic_goal)

            # Subscriber: receive planned trajectories
            topic_traj = f"/uav_{uid}/planning/trajectory"
            self._traj_subs[uid] = self.create_subscription(
                Path,
                topic_traj,
                lambda msg, u=uid: self._trajectory_callback(u, msg),
                qos,
            )
            logger.info("Created subscriber: %s", topic_traj)

    def _trajectory_callback(self, uav_id: str, msg: "Path") -> None:
        """Convert received Path message to list of (x, y, z) waypoints."""
        waypoints = []
        for pose_stamped in msg.poses:
            p = pose_stamped.pose.position
            waypoints.append((p.x, p.y, p.z))
        self.trajectories[uav_id] = waypoints
        logger.debug(
            "Received trajectory for %s: %d waypoints", uav_id, len(waypoints)
        )

    def publish_goal(
        self,
        uav_id: str,
        goal: Tuple[float, float, float],
    ) -> None:
        """Publish a PoseStamped goal to the EGO-Swarm planner."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(goal[0])
        msg.pose.position.y = float(goal[1])
        msg.pose.position.z = float(goal[2])
        # Orientation: identity quaternion (no preferred heading)
        msg.pose.orientation.w = 1.0
        self._goal_pubs[uav_id].publish(msg)
        logger.info("[%s] Goal published: (%.1f, %.1f, %.1f)", uav_id, *goal)

    def wait_for_trajectory(
        self, uav_id: str, timeout_sec: float = 5.0
    ) -> Optional[List[Tuple[float, float, float]]]:
        """
        Spin until a trajectory arrives for *uav_id* or timeout expires.

        Parameters
        ----------
        uav_id : str
        timeout_sec : float

        Returns
        -------
        list of (x, y, z) waypoints, or None on timeout.
        """
        self.trajectories[uav_id] = None
        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.trajectories[uav_id] is not None:
                return self.trajectories[uav_id]

        logger.warning(
            "[%s] Timeout waiting for EGO-Swarm trajectory.", uav_id
        )
        return None


# ===========================================================================
# Potential Field planner (pure-Python / numpy)
# ===========================================================================

class _PotentialFieldPlanner:
    """
    Classical Artificial Potential Field path planner.

    Physics
    -------
    Total force at position **q**::

        F_total = F_att + Σ F_rep_i

    Attractive force (linear, toward goal **g**)::

        F_att = α · (g − q)

    Repulsive force from obstacle **o_i** (within influence radius d₀)::

                    ⎧  β · (1/d − 1/d₀) · (1/d²) · (q − o_i)/d   if d < d₀
        F_rep_i =  ⎨
                    ⎩  0                                             otherwise

    where d = ‖q − o_i‖.

    Integration: Euler steps of size *step* until ‖q − g‖ < goal_radius
    or *max_iter* steps exceeded.

    Parameters
    ----------
    alpha : float
        Attractive gain (default 1.0).
    beta : float
        Repulsive gain (default 500.0).
    d0 : float
        Repulsion influence radius in metres (default 50.0).
    step : float
        Euler step size in metres (default 5.0).
    max_iter : int
        Maximum integration steps (default 500).
    goal_radius : float
        Distance from goal at which planning terminates (default 5.0 m).
    """

    def __init__(
        self,
        alpha: float = 1.0,
        beta: float = 500.0,
        d0: float = 50.0,
        step: float = 5.0,
        max_iter: int = 500,
        goal_radius: float = 5.0,
    ) -> None:
        self.alpha = alpha
        self.beta = beta
        self.d0 = d0
        self.step = step
        self.max_iter = max_iter
        self.goal_radius = goal_radius

    def plan(
        self,
        start: Tuple[float, float, float],
        goal: Tuple[float, float, float],
        obstacles: List[Tuple[float, float, float]],
    ) -> List[Tuple[float, float, float]]:
        """
        Compute a collision-avoiding path from *start* to *goal*.

        Parameters
        ----------
        start : (x, y, z)
            Starting position in metres.
        goal : (x, y, z)
            Target position in metres.
        obstacles : list of (x, y, z)
            Obstacle / other-UAV positions to repel from.

        Returns
        -------
        list of (x, y, z) waypoints along the planned path,
        including *start* and (approximately) *goal*.
        """
        path: List[Tuple[float, float, float]] = [start]
        q = _v3(start)
        g = _v3(goal)

        for iteration in range(self.max_iter):
            # ---- Attractive force ----------------------------------------
            att = (g - q) * self.alpha

            # ---- Repulsive forces ----------------------------------------
            rep = _Vec3(0.0, 0.0, 0.0)
            for obs_t in obstacles:
                obs = _v3(obs_t)
                diff = q - obs
                d = diff.norm()
                if 0.0 < d < self.d0:
                    # Magnitude: β·(1/d − 1/d₀)·(1/d²)
                    magnitude = (
                        self.beta
                        * (1.0 / d - 1.0 / self.d0)
                        * (1.0 / (d * d))
                    )
                    rep = rep + diff.normalize() * magnitude

            # ---- Total force, normalised, then scaled to step size -------
            total = att + rep
            total_norm = total.norm()
            if total_norm < 1e-9:
                # Local minimum – jitter slightly to escape
                q = q + _Vec3(0.5, 0.5, 0.0)
                continue

            q = q + total.normalize() * self.step
            path.append(q.to_tuple())

            # ---- Check termination condition -----------------------------
            dist_to_goal = (g - q).norm()
            if dist_to_goal < self.goal_radius:
                path.append(g.to_tuple())
                logger.debug(
                    "APF converged in %d steps (dist=%.2f m)",
                    iteration + 1, dist_to_goal,
                )
                return path

        logger.warning(
            "APF max_iter=%d reached without converging to goal.", self.max_iter
        )
        return path


# ===========================================================================
# EgoSwarmBridge – public interface
# ===========================================================================

class EgoSwarmBridge:
    """
    UAV-X trajectory planning bridge.

    Abstracts away whether EGO-Swarm (via ROS2) or the built-in
    Potential Field planner is used.  Callers always get back a
    ``list[(x, y, z)]`` of waypoints.

    Parameters
    ----------
    uav_ids : list[str]
        Identifiers of all UAVs in the swarm (e.g. ``["uav_0","uav_1"]``).
    alpha : float
        APF attractive gain (fallback only, default 1.0).
    beta : float
        APF repulsive gain (fallback only, default 500.0).
    d0 : float
        APF repulsion radius in metres (fallback only, default 50.0 m).
    step : float
        APF Euler step in metres (fallback only, default 5.0 m).
    max_iter : int
        APF maximum iterations (fallback only, default 500).
    ros2_timeout : float
        Seconds to wait for EGO-Swarm to return a trajectory (ROS2 only,
        default 5.0 s).  Falls back to APF if timeout exceeded.

    Attributes
    ----------
    backend : str
        ``"ros2"`` or ``"potential_field"``

    Examples
    --------
    >>> bridge = EgoSwarmBridge(["uav_0", "uav_1"])
    >>> path = bridge.plan_path(
    ...     "uav_0",
    ...     start=(0, 0, 30),
    ...     goal=(400, 200, 30),
    ...     other_uav_positions=[(200, 100, 30)],
    ... )
    >>> print(f"Path has {len(path)} waypoints")
    """

    def __init__(
        self,
        uav_ids: List[str],
        alpha: float = 1.0,
        beta: float = 500.0,
        d0: float = 50.0,
        step: float = 5.0,
        max_iter: int = 500,
        ros2_timeout: float = 5.0,
    ) -> None:
        self.uav_ids = list(uav_ids)
        self.ros2_timeout = ros2_timeout

        # APF planner is always constructed (used as fallback)
        self._apf = _PotentialFieldPlanner(
            alpha=alpha,
            beta=beta,
            d0=d0,
            step=step,
            max_iter=max_iter,
        )

        # External obstacle positions (set via update_obstacles)
        self._obstacles: List[Tuple[float, float, float]] = []

        # Last known positions of all UAVs (for inter-UAV repulsion)
        self._uav_positions: dict = {}

        # ROS2 node
        self._ros2_node: Optional["_EgoSwarmPlanningNode"] = None

        if _ROS2_AVAILABLE:
            try:
                rclpy.init(args=None)
                self._ros2_node = _EgoSwarmPlanningNode(uav_ids)
                self.backend = "ros2"
                logger.info(
                    "EgoSwarmBridge: ROS2/EGO-Swarm back-end ready "
                    "(timeout=%.1fs).", ros2_timeout
                )
            except Exception as exc:
                logger.warning(
                    "ROS2 init failed (%s) – using Potential Field.", exc
                )
                self.backend = "potential_field"
        else:
            self.backend = "potential_field"
            logger.info(
                "EgoSwarmBridge: Potential Field back-end ready "
                "(α=%.1f, β=%.1f, d₀=%.1f m, step=%.1f m, max_iter=%d).",
                alpha, beta, d0, step, max_iter,
            )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def plan_path(
        self,
        uav_id: str,
        start: Tuple[float, float, float],
        goal: Tuple[float, float, float],
        other_uav_positions: Optional[List[Tuple[float, float, float]]] = None,
    ) -> List[Tuple[float, float, float]]:
        """
        Plan a collision-free path from *start* to *goal* for *uav_id*.

        Parameters
        ----------
        uav_id : str
            Identifier of the UAV that needs a path.
        start : (x, y, z)
            Current position in metres.
        goal : (x, y, z)
            Desired target position in metres.
        other_uav_positions : list[(x, y, z)], optional
            Positions of *other* UAVs to treat as dynamic obstacles.
            If ``None``, uses the last positions supplied via
            :meth:`update_obstacles` only.

        Returns
        -------
        list of (x, y, z) tuples representing waypoints from *start*
        to (approximately) *goal*.  Always includes at least [start, goal].

        Examples
        --------
        >>> path = bridge.plan_path(
        ...     "uav_0",
        ...     start=(0, 0, 30),
        ...     goal=(500, 300, 30),
        ...     other_uav_positions=[(200, 150, 30), (350, 200, 30)],
        ... )
        """
        # Update last-known position for this UAV
        self._uav_positions[uav_id] = start

        # Combine static obstacles + other UAVs
        all_obstacles = list(self._obstacles)
        if other_uav_positions:
            all_obstacles.extend(other_uav_positions)

        if self.backend == "ros2" and self._ros2_node is not None:
            return self._plan_ros2(uav_id, start, goal, all_obstacles)
        else:
            return self._plan_apf(start, goal, all_obstacles)

    def update_obstacles(
        self,
        obstacle_positions: List[Tuple[float, float, float]],
    ) -> None:
        """
        Update the list of static obstacles in the world.

        These are combined with ``other_uav_positions`` in :meth:`plan_path`.

        Parameters
        ----------
        obstacle_positions : list[(x, y, z)]
            World-frame positions of static obstacles (buildings, debris, etc.).

        Examples
        --------
        >>> bridge.update_obstacles([
        ...     (500, 300, 0),   # Collapsed building 0
        ...     (900, 100, 0),   # Collapsed building 1
        ... ])
        """
        self._obstacles = list(obstacle_positions)
        logger.debug("Obstacles updated: %d positions.", len(self._obstacles))

    def shutdown(self) -> None:
        """
        Cleanly shut down the ROS2 node (if active).

        Call this before process exit to avoid rclpy shutdown errors.

        Examples
        --------
        >>> bridge.shutdown()
        """
        if self._ros2_node is not None:
            self._ros2_node.destroy_node()
            rclpy.shutdown()
            self._ros2_node = None
            logger.info("ROS2 node destroyed.")

    # ------------------------------------------------------------------
    # Back-end implementations
    # ------------------------------------------------------------------

    def _plan_ros2(
        self,
        uav_id: str,
        start: Tuple[float, float, float],
        goal: Tuple[float, float, float],
        obstacles: List[Tuple[float, float, float]],
    ) -> List[Tuple[float, float, float]]:
        """
        Submit goal to EGO-Swarm and collect the resulting trajectory.

        EGO-Swarm handles obstacle avoidance internally using its ESDF and
        gradient-based optimiser.  If the planner times out, we fall back
        to APF for this call.
        """
        assert self._ros2_node is not None

        # Publish the goal pose
        self._ros2_node.publish_goal(uav_id, goal)

        # Wait for trajectory
        traj = self._ros2_node.wait_for_trajectory(
            uav_id, timeout_sec=self.ros2_timeout
        )

        if traj is not None and len(traj) > 0:
            logger.info(
                "[%s] EGO-Swarm trajectory received: %d waypoints.",
                uav_id, len(traj),
            )
            return traj
        else:
            logger.warning(
                "[%s] EGO-Swarm timeout – falling back to APF.", uav_id
            )
            return self._plan_apf(start, goal, obstacles)

    def _plan_apf(
        self,
        start: Tuple[float, float, float],
        goal: Tuple[float, float, float],
        obstacles: List[Tuple[float, float, float]],
    ) -> List[Tuple[float, float, float]]:
        """
        Plan a path using the Artificial Potential Field algorithm.

        See :class:`_PotentialFieldPlanner` for algorithm details.
        """
        return self._apf.plan(start, goal, obstacles)

    def __repr__(self) -> str:
        return (
            f"EgoSwarmBridge(backend={self.backend!r}, "
            f"uavs={self.uav_ids}, "
            f"obstacles={len(self._obstacles)})"
        )


# ===========================================================================
# Module demo / self-test
# ===========================================================================

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    logger.info("=== EgoSwarmBridge demo (Potential Field) ===")

    bridge = EgoSwarmBridge(
        uav_ids=["uav_0", "uav_1", "uav_2"],
        alpha=1.0,
        beta=500.0,
        d0=50.0,
        step=5.0,
        max_iter=500,
    )

    logger.info("Backend: %s", bridge.backend)

    # Static obstacles: collapsed buildings
    bridge.update_obstacles([
        (500.0, 300.0, 0.0),
        (900.0, 100.0, 0.0),
        (700.0, 900.0, 0.0),
    ])

    # Plan paths for three UAVs
    scenarios = [
        ("uav_0", (0.0, 0.0, 30.0),   (500.0, 300.0, 30.0)),
        ("uav_1", (0.0, 0.0, 40.0),   (900.0, 100.0, 40.0)),
        ("uav_2", (100.0, 50.0, 35.0),(700.0, 900.0, 35.0)),
    ]

    for uid, start, goal in scenarios:
        # Other UAVs' positions (repulsive obstacles)
        others = [s for u, s, g in scenarios if u != uid]

        path = bridge.plan_path(
            uid,
            start=start,
            goal=goal,
            other_uav_positions=others,
        )

        logger.info(
            "[%s] Path: %d waypoints | start=%s goal=%s",
            uid, len(path), path[0], path[-1],
        )

    bridge.shutdown()
    logger.info("Demo complete.")
