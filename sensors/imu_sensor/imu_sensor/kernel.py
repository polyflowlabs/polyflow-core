from common.polyflow_kernel import PolyflowKernel


class ImuKernel(PolyflowKernel):
    """
    An IMU (a BNO085): orientation, angular velocity and acceleration.

    Bound to a sensor PART on the robot by `sensor_id` (its placement id). The reading arrives through
    `update_reading(data)` — from the simulator in Studio (which plays the sensor's hardware adapter,
    computing it from the body the part is mounted on) and, on the robot, from the ROS wrapper with the
    adapter's `Imu` state. Same dict either way; the kernel can't tell them apart.
    """

    def setup(self):
        self.sensor_id = str(self.get_param("sensor_id", ""))
        self.last_reading = None

    def update_reading(self, data):
        if not self.should_run(trigger_info={"pin_id": "reading"}):
            return
        data = dict(data or {})
        self.last_reading = data
        self.emit("imu", {
            "orientation": dict(data.get("orientation") or {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}),
            "angular_velocity": dict(data.get("angular_velocity") or {"x": 0.0, "y": 0.0, "z": 0.0}),
            "linear_acceleration": dict(data.get("linear_acceleration") or {"x": 0.0, "y": 0.0, "z": 0.0}),
        })

    def process_input(self, pin_id: str, data: dict):
        pass
