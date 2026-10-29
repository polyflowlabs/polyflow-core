# Contact Sensor

A contact switch (a walker's foot): touching or not.

Publishes `std_msgs/Bool` on `contact`. Bind it to the sensor part with **Sensor** (`sensor_id`). In Studio the simulator plays the sensor's hardware adapter and computes the reading from the body the part is mounted on; on the robot the adapter publishes PRP `Contact` state and this node forwards it. The kernel is fed through `update_reading(data)` either way.
