"""BNO085 IMU over I2C: publishes sensor_msgs/Imu (`Imu` state).

Orientation is the game rotation vector — gravity-referenced, Z up, no magnetometer (so no drift-prone
heading jumps near motors). Acceleration includes gravity (specific force), as the simulator reports it.
"""

from typing import Any, Optional

from polyflow_adapter_sdk import HardwareAdapter, LEVEL_ERROR, LEVEL_OK


class Bno085ImuAdapter(HardwareAdapter):
    driver_name = "bno085_imu"
    command_types: list = []
    state_types = ["Imu"]

    def configure(self) -> None:
        self.address = int(self.params.get("i2c_address", 0x4A))
        self.rate_hz = float(self.params.get("rate_hz", 100.0))
        self._bno: Optional[Any] = None

    def start(self) -> None:
        try:
            import board
            import busio
            from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_GYROSCOPE
            from adafruit_bno08x.i2c import BNO08X_I2C

            i2c = busio.I2C(board.SCL, board.SDA, frequency=400_000)
            self._bno = BNO08X_I2C(i2c, address=self.address)
            for report in (BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_GYROSCOPE, BNO_REPORT_ACCELEROMETER):
                self._bno.enable_feature(report)
        except Exception as exc:
            self.report_status(LEVEL_ERROR, "bno085_open_failed", str(exc), {"address": self.address})
            return
        self.report_status(LEVEL_OK, "connected", f"BNO085 at 0x{self.address:02X}")
        self.schedule_poll(self.rate_hz, self._poll)

    def _poll(self) -> None:
        bno = self._bno
        if bno is None:
            return
        try:
            qi, qj, qk, qr = bno.game_quaternion
            gx, gy, gz = bno.gyro
            ax, ay, az = bno.acceleration
        except Exception as exc:
            self.report_status(LEVEL_ERROR, "bno085_read_failed", str(exc))
            return
        self.publish_state("Imu", {
            "header": {"frame_id": self.target_id},
            "orientation": {"x": float(qi), "y": float(qj), "z": float(qk), "w": float(qr)},
            "angular_velocity": {"x": float(gx), "y": float(gy), "z": float(gz)},
            "linear_acceleration": {"x": float(ax), "y": float(ay), "z": float(az)},
        })

    def shutdown(self) -> None:
        self._bno = None
