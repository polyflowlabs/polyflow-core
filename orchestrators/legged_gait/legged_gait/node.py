import asyncio

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint
from common.polyflow_node import PolyflowNode
from legged_gait.kernel import LeggedGaitKernel, LEGS


class LeggedGaitNode(PolyflowNode):
    """ROS wrapper for LeggedGaitKernel."""

    kernel_class = LeggedGaitKernel

    def __init__(self):
        super().__init__()

        self.register_input_pin("cmd_vel", Twist)
        self.register_input_pin("imu", Imu)
        self.register_input_pin("joint_states", JointState)
        for leg in LEGS:
            self.register_input_pin(f"contact_{leg}", Bool)

        self.register_output_pin("joint_commands", JointTrajectoryPoint)

        self.get_logger().info(
            f"LeggedGait | {len(self.kernel.legs)} legs | rate={self.kernel.rate_hz}Hz | cycle={self.kernel.cycle_s}s"
        )

    async def run_async(self):
        self.get_logger().info("LeggedGait ready")
        period = 1.0 / max(self.kernel.rate_hz, 1e-3)
        while True:
            self.kernel.poll()
            await asyncio.sleep(period)


def main(args=None):
    LeggedGaitNode.main(args)
