import asyncio

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from polyflow_msgs.msg import DisplayCommand, GamepadButtons

from common.polyflow_node import PolyflowNode, _dict_to_ros_msg
from face.kernel import FaceKernel


class FaceNode(PolyflowNode):
    """
    ROS wrapper for FaceKernel. The kernel's "hw_command" emission becomes a DisplayCommand on the PRP
    hardware command topic, addressed to the display's hardware row (display_id), which the display's
    adapter draws.
    """

    kernel_class = FaceKernel

    def __init__(self):
        super().__init__()

        self.register_input_pin("cmd_vel", Twist)
        self.register_input_pin("buttons", GamepadButtons)
        self.register_input_pin("set_expression", String)
        self.register_output_pin("expression", String)

        self._display_publisher = self.create_publisher(DisplayCommand, "/prp/hardware/cmd/display_command", 10)

        original_emit = self.kernel.on_emit

        def on_kernel_emit(pin_id, data):
            if pin_id == "hw_command":
                msg = _dict_to_ros_msg(data, DisplayCommand)
                msg.stamp = self.get_clock().now().to_msg()
                self._display_publisher.publish(msg)
                self._trace_pin("OUT", "hw_command", msg)
            elif original_emit:
                original_emit(pin_id, data)

        self.kernel.on_emit = on_kernel_emit
        self.get_logger().info(f"Face | display={self.kernel.display_id} | idle_after={self.kernel.idle_after_s}s")

    async def run_async(self):
        self.get_logger().info("Face ready")
        period = 1.0 / max(self.kernel.rate_hz, 1e-3)
        while True:
            self.kernel.poll()
            await asyncio.sleep(period)


def main(args=None):
    FaceNode.main(args)
