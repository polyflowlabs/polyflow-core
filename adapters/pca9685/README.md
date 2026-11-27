# polyflow-pca9685-adapter

Polyflow hardware adapter for **PCA9685** 16-channel PWM servo HATs (the Adafruit 16-Channel PWM/Servo
HAT and its clones), from the NXP PCA9685 datasheet.

Two drivers:

- **`pca9685_board`**: the host. Owns the chip on its I²C bus, sets the chip-wide PWM frequency once,
  and shares the chip with every servo on it. E-stop and shutdown turn every channel full off.
- **`pca9685_servo`**: one hobby servo on one channel, driven as a **joint**. It consumes
  `JointCommand` (positions, which is what a Joint Controller sends) and `ServoCommand`, and publishes
  `JointState`.
  A PWM servo has no feedback, so the state echoes the last commanded position.

## hardware.json example

```json
{
  "adapters": [{ "id": "p_hat_pca9685_board", "name": "pca9685_board", "params": { "i2c_bus": 1, "address": 64, "frequency_hz": 50 } }],
  "drivers": [{ "id": "pan", "name": "pca9685_servo", "host": "p_hat_pca9685_board", "params": { "channel": 0 } }]
}
```

The deploy derives `channel` from the wire: the HAT header the servo is plugged into.

## Mapping

Linear: servo travel `[angle_min_rad, angle_max_rad]` → `[pulse_min_us, pulse_max_us]` (default ±90° →
500–2500 µs). The joint's 0 is `zero_rad` of servo travel; `gear_ratio` servo radians per joint radian
(a horn on gears or a linkage); `invert` flips a servo mounted the other way. Commands beyond the
servo's travel are clamped, and the state reports where it actually went.

`DISABLE`, `ESTOP` and `ServoCommand(torque_enable=false)` stop the pulses on that channel, so the
servo goes limp. `ENABLE` resends the last position.

## Tests

```
PYTHONPATH=adapters/polyflow_adapter_sdk/src:adapters/pca9685/src python3 -m unittest discover -s adapters/pca9685/tests -t adapters/pca9685
```
