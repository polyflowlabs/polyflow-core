import asyncio
from typing import Optional

from sensor_msgs.msg import Range
from sensor_msgs.msg import Range as OutMsg

from common.polyflow_node import PolyflowNode, _ros_msg_to_dict, ros_hardware_topic_token
from range_sensor.kernel import RangeSensorKernel


class RangeSensorNode(PolyflowNode):
    """ROS wrapper: subscribes to the sensor adapter's PRP `Range` state and feeds it to the kernel."""

    kernel_class = RangeSensorKernel

    def __init__(self):
        super().__init__()
        self.register_output_pin("range", OutMsg)
        self._sub: Optional[object] = None
        sensor_id = self.kernel.sensor_id
        if sensor_id:
            topic = f"/prp/hardware/state/{ros_hardware_topic_token(sensor_id)}/range"
            self._sub = self.create_subscription(Range, topic, lambda msg: self.kernel.update_reading(_ros_msg_to_dict(msg)), 10)
            self.get_logger().info(f"Subscribed to PRP state topic: {topic}")
        else:
            self.get_logger().warning("No sensor_id configured; PRP state subscription skipped")

    async def run_async(self):
        while True:
            await asyncio.sleep(1.0)


def main(args=None):
    RangeSensorNode.main(args)
