
import math
import random

import rclpy
import py_trees
import py_trees_ros
import py_trees_ros.subscribers
import py_trees_ros.trees

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, TeleportAbsolute

# =====================================================================
# Constants
# =====================================================================

# Blackboard keys
BB_STATE       = "/turtle/state"
BB_POSE        = "/turtle/pose"
BB_INSPECT     = "/inspect/result"       # "OK" | "NG" | None

# Waypoints (turtlesim coordinate space: 0 ~ 11)
WAYPOINT_PICKUP  = (2.0, 2.0)    # Product pickup point
WAYPOINT_INSPECT = (5.5, 8.0)    # Inspection point
WAYPOINT_OK      = (9.0, 9.0)    # OK placement zone
WAYPOINT_NG      = (9.0, 2.0)    # NG placement zone
WAYPOINT_HOME    = (5.5, 5.5)    # Home (idle) position

ARRIVE_THRESHOLD = 0.3           # Arrival distance threshold

# Pen colors (R, G, B)
PEN_IDLE     = (255, 255, 255)   # White  — idle
PEN_CARRYING = (0, 200, 0)      # Green  — carrying product
PEN_OK       = (0, 100, 255)    # Blue   — OK placement
PEN_NG       = (255, 50, 50)    # Red    — NG placement


class TurtleState:
    IDLE       = "IDLE"
    PICKING    = "PICKING"
    INSPECTING = "INSPECTING"
    PLACING_OK = "PLACING_OK"
    PLACING_NG = "PLACING_NG"


# =====================================================================
# [py_trees_ros] Topic → Blackboard subscriber node
# =====================================================================

class PoseToBlackboard(py_trees_ros.subscribers.ToBlackboard):
    """
    Auto-syncs /turtle1/pose to the Blackboard.
    turtlesim Pose message fields: x, y, theta, linear_velocity, angular_velocity.
    """
    def __init__(self):
        super().__init__(
            name="Sub: Pose → BB",
            topic_name="/turtle1/pose",
            topic_type=Pose,
            blackboard_variables={BB_POSE: None},
            clearing_policy=py_trees.common.ClearingPolicy.NEVER,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )


# =====================================================================
# Condition Nodes
# =====================================================================

class CheckState(py_trees.behaviour.Behaviour):
    """Checks whether the turtle state on the Blackboard matches the expected value."""
    def __init__(self, name: str, expected: str):
        super().__init__(name)
        self.expected = expected
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(key=BB_STATE, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        current = self.bb.get(BB_STATE)
        if current == self.expected:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


# =====================================================================
# Action Node — Base Class
# =====================================================================

class TurtleActionBase(py_trees.behaviour.Behaviour):
    """
    Base class for action nodes that use turtlesim ROS2 resources.
    Receives the ROS2 node via kwargs['node'] in setup().
    (py_trees_ros.trees.BehaviourTree injects 'node' automatically.)
    """
    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError(
                f"didn't find 'node' in setup's kwargs [{self.name}]"
            ) from e

    def _get_pose(self) -> Pose | None:
        bb = py_trees.blackboard.Blackboard()
        return bb.get(BB_POSE)

    def _distance_to(self, tx: float, ty: float) -> float:
        pose = self._get_pose()
        if pose is None:
            return float('inf')
        return math.hypot(tx - pose.x, ty - pose.y)


# =====================================================================
# Action Node — NavigateTo (move to target using P control)
# =====================================================================

class NavigateTo(TurtleActionBase):
    """
    Proportional control node that drives the turtle to target (tx, ty).
    Publishes cmd_vel every tick and returns SUCCESS on arrival.
    """
    def __init__(self, name: str, tx: float, ty: float):
        super().__init__(name)
        self.tx = tx
        self.ty = ty
        self._pub = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._pub = self.node.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.logger.info(f"{self.name} setup complete.")

    def update(self) -> py_trees.common.Status:
        pose = self._get_pose()
        if pose is None:
            self.logger.warning("No pose available yet.")
            return py_trees.common.Status.RUNNING

        dx = self.tx - pose.x
        dy = self.ty - pose.y
        dist = math.hypot(dx, dy)

        if dist < ARRIVE_THRESHOLD:
            # Stop
            self._publish_twist(0.0, 0.0)
            self.logger.info(
                f"{self.name}: Arrived at ({self.tx:.1f}, {self.ty:.1f})"
            )
            return py_trees.common.Status.SUCCESS

        # Compute angle error and apply proportional control
        target_angle = math.atan2(dy, dx)
        angle_diff = self._normalize_angle(target_angle - pose.theta)

        linear = min(1.5, dist)        # Speed cap
        angular = 4.0 * angle_diff     # P gain

        self._publish_twist(linear, angular)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and self._pub:
            self._publish_twist(0.0, 0.0)

    def _publish_twist(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._pub.publish(msg)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


# =====================================================================
# Action Node — SetPenColor (service call)
# =====================================================================

class SetPenColor(TurtleActionBase):
    """
    Calls the set_pen service to change the turtle's pen color.
    Visually represents the current state (idle / carrying / placing).
    """
    def __init__(self, name: str, r: int, g: int, b: int, width: int = 3, off: bool = False):
        super().__init__(name)
        self.r = r
        self.g = g
        self.b = b
        self.width = width
        self.off = off
        self._client = None
        self._future = None

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._client = self.node.create_client(SetPen, "/turtle1/set_pen")
        self.logger.info(f"{self.name} setup complete.")

    def initialise(self):
        self._future = None
        if not self._client.service_is_ready():
            self.logger.warning(f"{self.name}: set_pen service not ready, skipping.")
            return
        req = SetPen.Request()
        req.r = self.r
        req.g = self.g
        req.b = self.b
        req.width = self.width
        req.off = 1 if self.off else 0
        self._future = self._client.call_async(req)

    def update(self) -> py_trees.common.Status:
        # If the service was not ready, skip and succeed
        if self._future is None:
            return py_trees.common.Status.SUCCESS
        if not self._future.done():
            return py_trees.common.Status.RUNNING
        self.logger.info(f"{self.name}: Pen set to ({self.r},{self.g},{self.b})")
        return py_trees.common.Status.SUCCESS


# =====================================================================
# Action Node — SetState (state transition)
# =====================================================================

class SetState(py_trees.behaviour.Behaviour):
    """Writes a new state value to the Blackboard."""
    def __init__(self, name: str, new_state: str):
        super().__init__(name)
        self.new_state = new_state
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(key=BB_STATE, access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        self.bb.set(BB_STATE, self.new_state)
        self.logger.info(f"State → {self.new_state}")
        return py_trees.common.Status.SUCCESS


# =====================================================================
# Action Node — Inspect (random OK/NG verdict)
# =====================================================================

class InspectProduct(py_trees.behaviour.Behaviour):
    """
    Simulates inspection: 80% chance OK, 20% chance NG.
    Stores the result in BB_INSPECT.
    """
    def __init__(self):
        super().__init__(name="Action: Inspect")
        self.bb = self.attach_blackboard_client(name="InspectProduct")
        self.bb.register_key(key=BB_INSPECT, access=py_trees.common.Access.WRITE)
        self.bb.register_key(key=BB_STATE, access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        verdict = "OK" if random.random() < 0.8 else "NG"
        self.bb.set(BB_INSPECT, verdict)
        if verdict == "OK":
            self.bb.set(BB_STATE, TurtleState.PLACING_OK)
        else:
            self.bb.set(BB_STATE, TurtleState.PLACING_NG)
        self.logger.info(f"Inspection result: {verdict}")
        return py_trees.common.Status.SUCCESS


# =====================================================================
# Action Node — ResetCycle
# =====================================================================

class ResetCycle(TurtleActionBase):
    """
    Resets state after one cycle completes.
    Teleports the turtle to HOME and restores the pen to white.
    """
    def __init__(self):
        super().__init__(name="Action: Reset Cycle")
        self.bb = self.attach_blackboard_client(name="ResetCycle")
        self.bb.register_key(key=BB_STATE, access=py_trees.common.Access.WRITE)
        self.bb.register_key(key=BB_INSPECT, access=py_trees.common.Access.WRITE)
        self._tp_client = None
        self._pen_client = None
        self._tp_future = None
        self._pen_future = None
        self._step = 0

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._tp_client = self.node.create_client(
            TeleportAbsolute, "/turtle1/teleport_absolute"
        )
        self._pen_client = self.node.create_client(SetPen, "/turtle1/set_pen")
        self.logger.info("ResetCycle setup complete.")

    def initialise(self):
        self._step = 0
        self._tp_future = None
        self._pen_future = None

        # Turn pen OFF so no trail is drawn during teleport
        if self._pen_client.service_is_ready():
            req = SetPen.Request()
            req.r, req.g, req.b = 255, 255, 255
            req.width = 3
            req.off = 1
            self._pen_future = self._pen_client.call_async(req)

    def update(self) -> py_trees.common.Status:
        if self._step == 0:
            # Wait for pen OFF to complete (skip if service was not ready)
            if self._pen_future is not None and not self._pen_future.done():
                return py_trees.common.Status.RUNNING
            # Request teleport
            if self._tp_client.service_is_ready():
                req = TeleportAbsolute.Request()
                req.x, req.y = WAYPOINT_HOME
                req.theta = 0.0
                self._tp_future = self._tp_client.call_async(req)
            self._step = 1
            return py_trees.common.Status.RUNNING

        if self._step == 1:
            # Wait for teleport to complete
            if self._tp_future is not None and not self._tp_future.done():
                return py_trees.common.Status.RUNNING
            # Turn pen ON (white)
            if self._pen_client.service_is_ready():
                req = SetPen.Request()
                req.r, req.g, req.b = PEN_IDLE
                req.width = 3
                req.off = 0
                self._pen_future = self._pen_client.call_async(req)
            self._step = 2
            return py_trees.common.Status.RUNNING

        if self._step == 2:
            if self._pen_future is not None and not self._pen_future.done():
                return py_trees.common.Status.RUNNING
            # Clear Blackboard
            self.bb.set(BB_STATE, TurtleState.IDLE)
            self.bb.set(BB_INSPECT, None)
            self.logger.info("Cycle reset — teleported home. Ready for next cycle.")
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE


# =====================================================================
# Blackboard Initialization
# =====================================================================

def init_blackboard():
    client = py_trees.blackboard.Client(name="initialiser")
    client.register_key(BB_STATE, access=py_trees.common.Access.WRITE)
    client.register_key(BB_POSE, access=py_trees.common.Access.WRITE)
    client.register_key(BB_INSPECT, access=py_trees.common.Access.WRITE)
    client.set(BB_STATE, TurtleState.IDLE)
    client.set(BB_POSE, None)
    client.set(BB_INSPECT, None)


# =====================================================================
# Behavior Tree Assembly
# =====================================================================

def create_tree() -> py_trees.behaviour.Behaviour:
    """
    Tree structure:

    Root [Parallel: SuccessOnAll]
    ├── Sub: Pose → BB                       ← Always updates pose
    └── Main Selector (memory=False)
        ├── Seq: Phase 1 — Pick
        │   ├── CheckState(IDLE)
        │   ├── SetPenColor(white, OFF)       ← Hide travel trail
        │   ├── NavigateTo(PICKUP)
        │   ├── SetPenColor(green, ON)        ← Mark carrying start
        │   └── SetState(PICKING)
        │
        ├── Seq: Phase 2 — Move to inspect & inspect
        │   ├── CheckState(PICKING)
        │   ├── NavigateTo(INSPECT)
        │   ├── SetState(INSPECTING)
        │   └── InspectProduct               ← 80% OK / 20% NG
        │
        ├── Seq: Phase 3a — Place OK
        │   ├── CheckState(PLACING_OK)
        │   ├── SetPenColor(blue)
        │   ├── NavigateTo(OK zone)
        │   └── ResetCycle
        │
        └── Seq: Phase 3b — Place NG
            ├── CheckState(PLACING_NG)
            ├── SetPenColor(red)
            ├── NavigateTo(NG zone)
            └── ResetCycle
    """

    # ── Root: Parallel
    root = py_trees.composites.Parallel(
        name="Root",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )

    # ── Pose subscriber node
    pose_sub = PoseToBlackboard()

    # ── Main Selector
    main_sel = py_trees.composites.Selector(
        name="Main Selector", memory=False
    )

    # ── Phase 1: IDLE → move to PICKUP → PICKING
    seq_pick = py_trees.composites.Sequence(
        name="Seq: Phase1 Pick", memory=True
    )
    seq_pick.add_children([
        CheckState("Cond: IDLE?", TurtleState.IDLE),
        SetPenColor("Pen: off for travel", *PEN_IDLE, off=True),
        NavigateTo("Nav → Pickup", *WAYPOINT_PICKUP),
        SetPenColor("Pen: carrying (green)", *PEN_CARRYING),
        SetState("State → PICKING", TurtleState.PICKING),
    ])

    # ── Phase 2: PICKING → move to INSPECT → inspect
    seq_inspect = py_trees.composites.Sequence(
        name="Seq: Phase2 Inspect", memory=True
    )
    seq_inspect.add_children([
        CheckState("Cond: PICKING?", TurtleState.PICKING),
        NavigateTo("Nav → Inspect", *WAYPOINT_INSPECT),
        SetState("State → INSPECTING", TurtleState.INSPECTING),
        InspectProduct(),
    ])

    # ── Phase 3a: Place OK
    seq_ok = py_trees.composites.Sequence(
        name="Seq: Phase3a Place OK", memory=True
    )
    seq_ok.add_children([
        CheckState("Cond: PLACING_OK?", TurtleState.PLACING_OK),
        SetPenColor("Pen: OK (blue)", *PEN_OK),
        NavigateTo("Nav → OK zone", *WAYPOINT_OK),
        ResetCycle(),
    ])

    # ── Phase 3b: Place NG
    seq_ng = py_trees.composites.Sequence(
        name="Seq: Phase3b Place NG", memory=True
    )
    seq_ng.add_children([
        CheckState("Cond: PLACING_NG?", TurtleState.PLACING_NG),
        SetPenColor("Pen: NG (red)", *PEN_NG),
        NavigateTo("Nav → NG zone", *WAYPOINT_NG),
        ResetCycle(),
    ])

    main_sel.add_children([seq_pick, seq_inspect, seq_ok, seq_ng])
    root.add_children([pose_sub, main_sel])
    return root


# =====================================================================
# Main
# =====================================================================

def main():
    rclpy.init()

    py_trees.blackboard.Blackboard.enable_activity_stream()
    init_blackboard()

    root = create_tree()

    # py_trees_ros BehaviourTree — integrates ROS2 spin and tick
    behaviour_tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True,
    )

    try:
        behaviour_tree.setup(timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        print(f"[ERROR] BT setup timed out: {e}")
        rclpy.shutdown()
        return

    print("\n" + "=" * 60)
    print("  Turtlesim Behavior Tree — Pick & Place Simulation")
    print("  Waypoints:")
    print(f"    PICKUP  : {WAYPOINT_PICKUP}")
    print(f"    INSPECT : {WAYPOINT_INSPECT}")
    print(f"    OK zone : {WAYPOINT_OK}")
    print(f"    NG zone : {WAYPOINT_NG}")
    print(f"    HOME    : {WAYPOINT_HOME}")
    print("=" * 60 + "\n")

    try:
        behaviour_tree.tick_tock(period_ms=100)   # 10Hz tick
        rclpy.spin(behaviour_tree.node)
    except KeyboardInterrupt:
        pass
    finally:
        behaviour_tree.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
