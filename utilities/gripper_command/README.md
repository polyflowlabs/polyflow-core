# Gripper Command

Turns a button into a gripper position. Press to close, press again to open (`toggle`), or close while
held (`hold`). It emits a `JointTrajectoryPoint` for the gripper's Joint Controller, so a gamepad can
work a gripper with no positions to tune: **open** and **closed** are read off the gripper joint's own
travel limits (lower and upper).

## Pins

| Pin | Direction | Type | Description |
| --- | --- | --- | --- |
| `button` | subscribe | `std_msgs/Bool` | A button's pressed state, from Bool From Gamepad. |
| `trajectory` | publish | `trajectory_msgs/JointTrajectoryPoint` | The gripper joint's open or closed position. Wire into the gripper's Joint Controller `trajectory`. |
| `closed` | publish | `std_msgs/Bool` | Whether the gripper is commanded closed. |

## Parameters

- **Gripper Joint** (`joint_id`): the gripper's *driven* joint (the one its motor turns), not a jaw
  that mimics it.
- **Open Position** / **Closed Position**: bound to that joint's lower / upper limit.
- **Button Mode** (`mode`): `toggle` (default) or `hold`.
- **Start Closed** (`start_closed`): where a toggle starts.
- **Invert** (`invert`): swap open and closed.

## Typical use

`Gamepad.buttons` → `Bool From Gamepad` (button `a`) → `Gripper Command.button`, then
`Gripper Command.trajectory` → the gripper's `Joint Controller.trajectory`. Studio's arm control
template wires exactly this.
