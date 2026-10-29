# polyflow-sensors-adapter

Hardware adapters for the sensor nodes (`IMU`, `Contact Sensor`, `Range Sensor`):

| Driver | Device | State | Message |
| --- | --- | --- | --- |
| `bno085_imu` | BNO085 IMU (I2C) | `Imu` | `sensor_msgs/Imu` — game rotation vector (Z up), gyro, acceleration incl. gravity |
| `gpio_contact` | a switch to ground on a GPIO pin (pulled up) | `Contact` | `std_msgs/Bool` — true while pressed |
| `vl53l1x_range` | VL53L1X time-of-flight (I2C) | `Range` | `sensor_msgs/Range` — max_range when nothing is in reach |

Each publishes the same dict the Studio simulator computes for that sensor, so the nodes run unchanged
in the scene and on the robot. `system-manager` must map the `Contact` and `Range` state types to
`std_msgs/Bool` and `sensor_msgs/Range` (it already maps `Imu`).
