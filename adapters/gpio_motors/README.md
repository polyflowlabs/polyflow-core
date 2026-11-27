# polyflow-gpio-motors-adapter

Polyflow hardware adapter for motor drivers wired straight to a Raspberry Pi's 40-pin GPIO header.
Built on **lgpio**, which works on a Pi 5 (RPi.GPIO doesn't). There is no host driver: every driver
opens the same chip through one shared, per-process handle, and two drivers that are given the same pin
are refused at start-up (`pin_unavailable`) instead of fighting over the wire.

Three drivers, each running one **joint**:

| Driver | Hardware | Pins | Commands → state |
|---|---|---|---|
| `tb6612_motor` | one channel of a TB6612FNG dual H-bridge, brushed DC motor | `pwm_pin`, `in1_pin`, `in2_pin` (+ optional `stby_pin`) | `MotorCommand` → `MotorState` |
| `stepstick_stepper` | A4988 / DRV8825 / TMC2208 / TMC2209 step/dir driver, bipolar stepper | `step_pin`, `dir_pin` (+ optional `enable_pin`) | `JointCommand` → `JointState` |
| `gpio_servo` | hobby PWM servo, signal wire on a GPIO | `signal_pin` | `JointCommand`, `ServoCommand` → `JointState` |

Pins are BCM numbers (`GPIO12`, not header pin 32). Studio's deploy assigns them and its build
instructions list which header pin goes to which driver terminal.

## hardware.json example

```json
{
  "adapters": [],
  "drivers": [
    { "id": "left_wheel", "name": "tb6612_motor", "params": { "pwm_pin": 12, "in1_pin": 5, "in2_pin": 6, "max_speed_rad_s": 12 } },
    { "id": "shoulder", "name": "stepstick_stepper", "params": { "step_pin": 17, "dir_pin": 27, "microsteps": 16, "gear_ratio": 20 } },
    { "id": "gripper", "name": "gpio_servo", "params": { "signal_pin": 18 } }
  ]
}
```

## TB6612FNG

From the datasheet's truth table: IN1/IN2 set direction, PWM sets duty, H/H short-brakes, L/L coasts.
A Motor Controller's SPEED (rad/s) is scaled to duty by `max_speed_rad_s`, the speed at 100% duty,
which the deploy sets to the joint's top speed so a full stick is full duty. There is no encoder input,
so the drive is open loop and the state echoes the command. A command's `timeout_s` is a watchdog.

Wire STBY to 3.3 V. If it is on a pin, e-stop drops it, which stops both channels.

## StepStick

One STEP pulse is one microstep. Motion is a trapezoidal profile (`StepPlanner`) run at `control_hz`.
Each tick queues that tick's whole steps as one evenly spaced pulse train (lgpio `tx_pulse`).

    microsteps per joint radian = steps_per_rev × microsteps × gear_ratio / 2π

The position is **counted, not measured**: the joint reads 0 where it sat at power-on, so start the
robot in its zero pose. A motor that stalls skips steps and the count is then wrong.

E-stop stops the pulses and keeps the coils energized, so an arm holds rather than dropping. It stays
latched until `ENABLE`. With `enable_pin` set (active low), `DISABLE` de-energizes the motor.

Wire EN to GND. Set the microstep pins to 1/16, which is the default `microsteps`:

| Driver | MS pins for 1/16 |
|---|---|
| A4988 | MS1, MS2, MS3 high |
| DRV8825 | M2 high, M0 and M1 low |
| TMC2208 / TMC2209 (standalone) | MS1 and MS2 high |

On the A4988 and DRV8825, join RESET to SLEEP.

## GPIO servo

The same mapping as `pca9685_servo`. Software-timed pulses jitter a little, so for more than a couple
of servos a PCA9685 HAT is the better part. Power the servo from its own supply, not the Pi's 5 V pin.

## Runtime dependency

`lgpio` (and its `liblgpio`) must be in the robot's system-manager Python environment.

## Tests

```
PYTHONPATH=adapters/polyflow_adapter_sdk/src:adapters/gpio_motors/src python3 -m unittest discover -s adapters/gpio_motors/tests -t adapters/gpio_motors
```
