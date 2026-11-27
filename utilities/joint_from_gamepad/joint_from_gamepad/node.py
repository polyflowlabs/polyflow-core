import asyncio

from polyflow_msgs.msg import GamepadAxes
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from common.polyflow_node import PolyflowNode
from joint_from_gamepad.kernel import JointFromGamepadKernel


class JointFromGamepadNode(PolyflowNode):
    """ROS wrapper for JointFromGamepadKernel."""

    kernel_class = JointFromGamepadKernel

    def __init__(self):
        super().__init__()

        self.register_input_pin("axes", GamepadAxes)
        self.register_input_pin("joint_state", JointState)
        self.register_output_pin("trajectory", JointTrajectoryPoint)

        self.get_logger().info(
            f"JointFromGamepad | joint={self.kernel.joint_id} | axis={self.kernel.axis} | "
            f"speed={self.kernel.speed} | rate={self.kernel.rate_hz}Hz"
        )

    async def run_async(self):
        self.get_logger().info("JointFromGamepad ready")
        period = 1.0 / max(self.kernel.rate_hz, 1e-3)
        while True:
            self.kernel.poll()
            await asyncio.sleep(period)


def main(args=None):
    JointFromGamepadNode.main(args)
