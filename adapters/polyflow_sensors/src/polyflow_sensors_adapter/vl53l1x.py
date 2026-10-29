"""VL53L1X time-of-flight range sensor over I2C: publishes sensor_msgs/Range (`Range` state).
Out of range reads as max_range, as the simulator reports a ray that hits nothing."""

from typing import Any, Optional

from polyflow_adapter_sdk import HardwareAdapter, LEVEL_ERROR, LEVEL_OK

MIN_RANGE_M = 0.04
MAX_RANGE_M = {1: 1.3, 2: 4.0}
FIELD_OF_VIEW_RAD = 0.47   # 27°


class Vl53l1xRangeAdapter(HardwareAdapter):
    driver_name = "vl53l1x_range"
    command_types: list = []
    state_types = ["Range"]

    def configure(self) -> None:
        self.address = int(self.params.get("i2c_address", 0x29))
        self.rate_hz = float(self.params.get("rate_hz", 50.0))
        self.mode = int(self.params.get("distance_mode", 2))
        self._tof: Optional[Any] = None

    def start(self) -> None:
        try:
            import adafruit_vl53l1x
            import board

            self._tof = adafruit_vl53l1x.VL53L1X(board.I2C(), address=self.address)
            self._tof.distance_mode = self.mode
            self._tof.timing_budget = 50 if self.rate_hz <= 20 else 20
            self._tof.start_ranging()
        except Exception as exc:
            self.report_status(LEVEL_ERROR, "vl53l1x_open_failed", str(exc), {"address": self.address})
            return
        self.report_status(LEVEL_OK, "connected", f"VL53L1X at 0x{self.address:02X}")
        self.schedule_poll(self.rate_hz, self._poll)

    def _poll(self) -> None:
        tof = self._tof
        if tof is None or not tof.data_ready:
            return
        cm = tof.distance
        tof.clear_interrupt()
        max_m = MAX_RANGE_M.get(self.mode, 4.0)
        dist = max_m if cm is None else min(max_m, max(MIN_RANGE_M, float(cm) / 100.0))
        self.publish_state("Range", {
            "header": {"frame_id": self.target_id},
            "radiation_type": 1,
            "field_of_view": FIELD_OF_VIEW_RAD,
            "min_range": MIN_RANGE_M,
            "max_range": max_m,
            "range": dist,
        })

    def shutdown(self) -> None:
        if self._tof is not None:
            try:
                self._tof.stop_ranging()
            except Exception:
                pass
            self._tof = None
