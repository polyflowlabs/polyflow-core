"""A contact switch on a GPIO pin (a walker's foot): publishes std_msgs/Bool (`Contact` state).
The switch closes to ground; the pin is pulled up, so pressed = in contact."""

from typing import Any, Optional

from polyflow_adapter_sdk import HardwareAdapter, LEVEL_ERROR, LEVEL_OK


class GpioContactAdapter(HardwareAdapter):
    driver_name = "gpio_contact"
    command_types: list = []
    state_types = ["Contact"]

    def configure(self) -> None:
        self.gpio = int(self.params["gpio"])
        self.rate_hz = float(self.params.get("rate_hz", 200.0))
        self._button: Optional[Any] = None

    def start(self) -> None:
        try:
            from gpiozero import Button

            self._button = Button(self.gpio, pull_up=True, bounce_time=None)
        except Exception as exc:
            self.report_status(LEVEL_ERROR, "gpio_open_failed", str(exc), {"gpio": self.gpio})
            return
        self.report_status(LEVEL_OK, "connected", f"contact switch on GPIO {self.gpio}")
        self.schedule_poll(self.rate_hz, self._poll)

    def _poll(self) -> None:
        if self._button is not None:
            self.publish_state("Contact", {"data": bool(self._button.is_pressed)})

    def shutdown(self) -> None:
        if self._button is not None:
            self._button.close()
            self._button = None
