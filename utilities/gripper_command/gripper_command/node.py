import asyncio

from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint
from common.polyflow_node import PolyflowNode
from gripper_command.kernel import GripperCommandKernel


class GripperCommandNode(PolyflowNode):
    """ROS wrapper for GripperCommandKernel."""

    kernel_class = GripperCommandKernel

    def __init__(self):
        super().__init__()

        self.register_input_pin("button", Bool)
        self.register_output_pin("trajectory", JointTrajectoryPoint)
        self.register_output_pin("closed", Bool)

        self.get_logger().info(f"GripperCommand | joint={self.kernel.joint_id} | mode={self.kernel.mode}")

    async def run_async(self):
        self.get_logger().info("GripperCommand ready")
        while True:
            await asyncio.sleep(1.0)


def main(args=None):
    GripperCommandNode.main(args)
