# Range Sensor

A distance sensor (a VL53L1X time-of-flight): metres to the first thing ahead.

Publishes `sensor_msgs/Range` on `range`. Bind it to the sensor part with **Sensor** (`sensor_id`). In Studio the simulator plays the sensor's hardware adapter and computes the reading from the body the part is mounted on; on the robot the adapter publishes PRP `Range` state and this node forwards it. The kernel is fed through `update_reading(data)` either way.
