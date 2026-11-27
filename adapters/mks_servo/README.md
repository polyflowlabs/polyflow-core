# polyflow-mks-servo-adapter

Polyflow hardware adapter for the Makerbase **MKS SERVO42D / 57D / 28D** closed-loop stepper drivers
over CAN, from their CAN User Manual V1.0.9 (firmware V1.0.9).

One driver, **`mks_servo_can`**: one adapter instance per joint, each opening its own SocketCAN socket
filtered to its driver's CAN id (Linux shares the bus between sockets, as `odrive_can` does).

## hardware.json example

```json
{ "id": "shoulder", "name": "mks_servo_can", "params": { "can_id": 2, "channel": "can0", "gear_ratio": 26.85 } }
```

The deploy derives `can_id` and `channel` from the robot's harness: the HAT channel the joint bus
starts at, and each driver's place along it (1 for the first, then 2, 3 …). `gear_ratio` comes from the
joint's gearbox or transmission.

## Setting up each driver (once, on its screen)

- **Mode**: anything — the adapter switches it to `SR_vFOC` (bus FOC) at startup.
- **CanID**: the number the deploy gave this joint (the build instructions list them).
- **CanRate**: **1M**. Polyflow OS brings `can0`/`can1` up at 1 Mbit/s (the drivers ship at 500K).
- Terminate the bus at both ends: the HAT and the last driver's 120 Ω jumper.

Power the robot up in its home pose: the power-on position is each joint's zero (or set
`zero_on_start`).

## Commands consumed

- `JointCommand`: **position** (an F5 absolute-coordinate move, which retargets mid-move, so a stream
  of targets follows smoothly; speed from the command's velocity, else `move_speed_rpm`) and
  **velocity** (F6 speed mode). Torque isn't supported by the drivers and is reported once.
- `HardwareLifecycle`: ENABLE / DISABLE (F3) and ESTOP (F7).

## State published

- `JointState`: position from the accumulated encoder (0x31), velocity from the speed read (0x32),
  both in joint units through `gear_ratio` and `invert`; `connected` while the driver answers.

## Direction

The manual contradicts itself about which way counts up. This adapter follows its worked examples
(CCW positive). If a joint moves the wrong way on first power-up, set `invert`.

## Tests

```
PYTHONPATH=adapters/polyflow_adapter_sdk/src:adapters/mks_servo/src python3 -m unittest discover -s adapters/mks_servo/tests -t adapters/mks_servo
```

They run against a simulated driver that speaks the manual's protocol, and check every frame the
adapter sends against the manual's own worked examples.
