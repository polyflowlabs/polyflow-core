from common.polyflow_kernel import PolyflowKernel


class RangeSensorKernel(PolyflowKernel):
    """
    A distance sensor (a VL53L1X time-of-flight): metres to the first thing ahead.

    Bound to a sensor PART on the robot by `sensor_id` (its placement id). The reading arrives through
    `update_reading(data)` — from the simulator in Studio (which plays the sensor's hardware adapter,
    computing it from the body the part is mounted on) and, on the robot, from the ROS wrapper with the
    adapter's `Range` state. Same dict either way; the kernel can't tell them apart.
    """

    def setup(self):
        self.sensor_id = str(self.get_param("sensor_id", ""))
        self.last_reading = None

    def update_reading(self, data):
        if not self.should_run(trigger_info={"pin_id": "reading"}):
            return
        data = dict(data or {})
        self.last_reading = data
        self.emit("range", {
            "radiation_type": 1,  # INFRARED — a time-of-flight laser
            "field_of_view": float(self.get_param("field_of_view_rad", 0.47)),
            "min_range": float(data.get("min_range", 0.0)),
            "max_range": float(data.get("max_range", 0.0)),
            "range": float(data.get("range", 0.0)),
        })

    def process_input(self, pin_id: str, data: dict):
        pass
