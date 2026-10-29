import asyncio

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from common.polyflow_node import PolyflowNode
from mecanum_drive.kernel import MecanumDriveKernel


class MecanumDriveNode(PolyflowNode):
    """ROS wrapper for MecanumDriveKernel."""

    kernel_class = MecanumDriveKernel

    def __init__(self):
        super().__init__()

        self.register_input_pin("cmd_vel_teleop", Twist)
        self.register_input_pin("cmd_vel_automated", Twist)

        self.register_output_pin("front_left_motor", Float64)
        self.register_output_pin("front_right_motor", Float64)
        self.register_output_pin("back_left_motor", Float64)
        self.register_output_pin("back_right_motor", Float64)

        self.get_logger().info(
            f"MecanumDrive | teleop_timeout_s={self.kernel.teleop_timeout_s}"
        )

    async def run_async(self):
        self.get_logger().info("MecanumDrive ready")

        # Publish zero commands on startup (the base starts stopped)
        self.kernel._emit_motor_commands((0.0, 0.0, 0.0, 0.0))

        while True:
            await asyncio.sleep(1.0)


def main(args=None):
    MecanumDriveNode.main(args)
