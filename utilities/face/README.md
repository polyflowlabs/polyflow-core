# Face

A robot's face, from what it's doing. It sends a `DisplayCommand` (expression and gaze) to the robot's
screen, whose adapter (`oled_display`) draws the eyes and animates them between commands.

## Pins

| Pin | Direction | Type | Description |
| --- | --- | --- | --- |
| `cmd_vel` | subscribe | `geometry_msgs/Twist` | What the robot is told to drive. The eyes look into a turn, up going forward, down in reverse. |
| `buttons` | subscribe | `polyflow_msgs/GamepadButtons` | Each button flashes an expression: a happy, b surprised, x sad, y angry. |
| `set_expression` | subscribe | `std_msgs/String` | A standing expression; an empty string hands control back. |
| `expression` | publish | `std_msgs/String` | The expression being shown. |

## Parameters

- **Display** (`display_id`): the screen's placement on the robot.
- **Sleepy After** (`idle_after_s`, 8 s): no driving for this long and the eyes go sleepy.
- **Button Expression For** (`flash_s`, 2 s).
- **Look Into Turns** (`look_gain`, 1).

## Typical use

`Gamepad.cmd_vel` and `Gamepad.buttons` → `Face`. Studio's drive templates wire this when the robot has a
screen.
