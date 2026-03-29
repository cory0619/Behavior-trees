# Behavior-trees

A turtlesim Pick & Place simulation built on ROS2 + py_trees.  
The turtle uses a Behavior Tree to repeatedly pick up a product, inspect it, and route it to the OK or NG zone based on the result.

---

## File Structure

| File | Description |
|------|-------------|
| `bt_turtlesim.py` | Main Behavior Tree code |

---

## Dependencies

| Package | Purpose |
|---------|---------|
| `rclpy` | ROS2 Python client library |
| `py_trees` | Behavior Tree library |
| `py_trees_ros` | py_trees ↔ ROS2 integration |
| `turtlesim` | Turtle simulator |

---

## System Overview

The turtle cycles through 5 states to perform a Pick & Place loop.

```
IDLE → (move to pickup) → PICKING → (move to inspect) → INSPECTING
     → PLACING_OK / PLACING_NG → (move to zone) → IDLE (repeat)
```

### Waypoints (turtlesim coordinate space: 0 ~ 11)

| Name | Coordinates | Description |
|------|-------------|-------------|
| PICKUP | (2.0, 2.0) | Product pickup point |
| INSPECT | (5.5, 8.0) | Inspection point |
| OK zone | (9.0, 9.0) | Placement zone for OK products |
| NG zone | (9.0, 2.0) | Placement zone for NG products |
| HOME | (5.5, 5.5) | Idle/standby position after cycle |

### Pen Colors

| Color | State | Meaning |
|-------|-------|---------|
| White | IDLE / before travel | Trail hidden |
| Green | PICKING | Carrying product |
| Blue | PLACING_OK | OK product placement |
| Red | PLACING_NG | NG product placement |

---

## Behavior Tree Structure

```
Root [Parallel: SuccessOnAll]
├── Sub: Pose → BB                      ← Live subscription to /turtle1/pose
└── Main Selector
    ├── Seq: Phase 1 — Pick
    │   ├── CheckState(IDLE)
    │   ├── SetPenColor(white, OFF)      ← Hide travel trail
    │   ├── NavigateTo(PICKUP)
    │   ├── SetPenColor(green, ON)
    │   └── SetState(PICKING)
    │
    ├── Seq: Phase 2 — Inspect
    │   ├── CheckState(PICKING)
    │   ├── NavigateTo(INSPECT)
    │   ├── SetState(INSPECTING)
    │   └── InspectProduct              ← Random verdict: 80% OK / 20% NG
    │
    ├── Seq: Phase 3a — Place OK
    │   ├── CheckState(PLACING_OK)
    │   ├── SetPenColor(blue)
    │   ├── NavigateTo(OK zone)
    │   └── ResetCycle                  ← Teleport home & reset blackboard
    │
    └── Seq: Phase 3b — Place NG
        ├── CheckState(PLACING_NG)
        ├── SetPenColor(red)
        ├── NavigateTo(NG zone)
        └── ResetCycle
```

---

## Key Classes

### Subscriber Node
- **`PoseToBlackboard`** — Subscribes to `/turtle1/pose` and auto-syncs to Blackboard key `/turtle/pose`.

### Condition Node
- **`CheckState`** — Returns SUCCESS if the current turtle state on the Blackboard matches the expected value, otherwise FAILURE.

### Action Nodes
| Class | Description |
|-------|-------------|
| `NavigateTo` | Moves to a target coordinate using proportional (P) control. Publishes to `/turtle1/cmd_vel` each tick. Returns SUCCESS on arrival. |
| `SetPenColor` | Calls the `/turtle1/set_pen` service to set pen color, width, and ON/OFF state. |
| `SetState` | Writes a new state value to Blackboard key `/turtle/state`. |
| `InspectProduct` | Simulates inspection: 80% OK, 20% NG. Stores result in Blackboard key `/inspect/result`. |
| `ResetCycle` | Resets the cycle: pen OFF → teleport HOME → pen ON (white) → clear Blackboard. |

---

## How to Run

### 1. Launch turtlesim

```bash
ros2 run turtlesim turtlesim_node
```

### 2. Run the Behavior Tree

```bash
python3 bt_turtlesim.py
```

> Tick period: **100 ms (10 Hz)**

---

## Blackboard Keys

| Key | Type | Description |
|-----|------|-------------|
| `/turtle/state` | `str` | Current turtle state (`IDLE`, `PICKING`, `INSPECTING`, `PLACING_OK`, `PLACING_NG`) |
| `/turtle/pose` | `turtlesim/Pose` | Current turtle position and orientation |
| `/inspect/result` | `str` \| `None` | Inspection result (`"OK"`, `"NG"`, `None`) |
