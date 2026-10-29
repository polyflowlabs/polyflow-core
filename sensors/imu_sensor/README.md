# IMU

An IMU (a BNO085): orientation, angular velocity and acceleration.

Publishes `sensor_msgs/Imu` on `imu`. Bind it to the sensor part with **Sensor** (`sensor_id`). In Studio the simulator plays the sensor's hardware adapter and computes the reading from the body the part is mounted on; on the robot the adapter publishes PRP `Imu` state and this node forwards it. The kernel is fed through `update_reading(data)` either way.
