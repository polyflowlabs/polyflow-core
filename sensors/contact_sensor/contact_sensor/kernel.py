from common.polyflow_kernel import PolyflowKernel


class ContactSensorKernel(PolyflowKernel):
    """
    A contact switch (a walker's foot): touching or not.

    Bound to a sensor PART on the robot by `sensor_id` (its placement id). The reading arrives through
    `update_reading(data)` — from the simulator in Studio (which plays the sensor's hardware adapter,
    computing it from the body the part is mounted on) and, on the robot, from the ROS wrapper with the
    adapter's `Contact` state. Same dict either way; the kernel can't tell them apart.
    """

    def setup(self):
        self.sensor_id = str(self.get_param("sensor_id", ""))
        self.last_reading = None

    def update_reading(self, data):
        if not self.should_run(trigger_info={"pin_id": "reading"}):
            return
        data = dict(data or {})
        self.last_reading = data
        touching = data.get("in_contact", data.get("data", False))
        self.emit("contact", {"data": bool(touching)})

    def process_input(self, pin_id: str, data: dict):
        pass
