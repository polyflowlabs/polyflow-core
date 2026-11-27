# Joint From Gamepad

Moves one joint from one gamepad stick axis. Hold the stick and the joint turns at a speed proportional
to how far it's pushed. Let go and it stops where it is. It emits a `JointTrajectoryPoint` (a position)
for the joint's Joint Controller, so a pan-tilt head, a turret or a lift can be aimed by hand on the
real robot.

- **The stick is a speed.** Each tick the target moves by `speed × stick × dt`, clamped to the joint's own
  travel limits. Stick right and stick up are positive; `invert` flips it.
- **At rest it sends nothing.** A slider or a sim test can still move the joint. The next push starts
  from where the joint really is (its `joint_state`), not from where the node last left its target.
- **No wind-up.** While moving, the target stays within `max_lead` of the joint's reported position. A
  joint that lags behind stops soon after the stick is released instead of carrying on.

## Pins

| Pin | Direction | Type | Description |
| --- | --- | --- | --- |
| `axes` | subscribe | `polyflow_msgs/GamepadAxes` | The Gamepad's sticks. |
| `joint_state` | subscribe | `sensor_msgs/JointState` | The joint's Joint Controller `state`. |
| `trajectory` | publish | `trajectory_msgs/JointTrajectoryPoint` | The joint's target position, only while the stick is pushed. Wire into the Joint Controller's `trajectory`. |

## Parameters

- **Joint** (`joint_id`): the joint to move.
- **Stick Axis** (`axis`): `left_x` (default), `left_y`, `right_x` or `right_y`.
- **Speed** (`speed`): rad/s at full stick (default 1.0).
- **Invert** (`invert`): reverse the direction.
- **Min. / Max. Position**: bound to the joint's lower and upper limits.
- **Max. Lead** (`max_lead`): how far the target may run ahead of the joint, in rad (default 0.25; 0 = no limit).
- **Integration Rate** (`rate_hz`): default 50 Hz.

## Typical use

`Gamepad.axes` → `Joint From Gamepad.axes` → `.trajectory` → the joint's `Joint Controller.trajectory`,
and that controller's `state` → `Joint From Gamepad.joint_state`. Studio's `pan_tilt` control template
wires two of these: pan on the left stick's X, tilt on its Y.
