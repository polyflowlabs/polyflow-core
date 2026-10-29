# Mecanum Drive

4-wheel mecanum kinematics: drive forward, strafe sideways and turn, in any combination. Consumes `geometry_msgs/Twist` commands and emits a normalized command on each of four motor pins. The two Twist inputs implement the same priority arbiter as Differential Drive: teleop wins whenever a recent message has arrived; otherwise the automated input is used.

## Pins

| Pin | Direction | Type | Description |
| --- | --- | --- | --- |
| `cmd_vel_teleop` | subscribe | `geometry_msgs/Twist` | Operator command — the Gamepad in `6dof` mode (left stick = forward/sideways, right stick X = turn). Takes priority while fresh. |
| `cmd_vel_automated` | subscribe | `geometry_msgs/Twist` | Autonomous planner command. Used when teleop is stale or has never published. |
| `front_left_motor` | publish | `std_msgs/Float64` | `x − y − z`, normalized. |
| `front_right_motor` | publish | `std_msgs/Float64` | `x + y + z`, normalized. |
| `back_left_motor` | publish | `std_msgs/Float64` | `x + y − z`, normalized. |
| `back_right_motor` | publish | `std_msgs/Float64` | `x − y + z`, normalized. |

`x` = `linear.x` (forward), `y` = `linear.y` (left), `z` = `angular.z` (turn left), each in [-1, 1]. When a wheel's command would exceed 1, all four are scaled down together, so the direction of travel is kept.

## Wheels

The mix assumes the rollers form an **X seen from above**: front-left and back-right are one hand, front-right and back-left the other. Every wheel's positive command must drive the robot forward — reverse the motors mounted mirrored, as on any skid-steer base.

## Parameters

- **Teleop Timeout** (`teleop_timeout_s`) — seconds without a teleop message before falling back to the automated input. Default `1.0`.
