# Legged Gait

A statically stable crawl for a four-legged robot: the body shifts its centre of mass over the
three feet that stay down before each leg lifts, so it never relies on balance it doesn't have.

- Finds the legs from the robot's bodies: each leg runs from the base body to a body with a foot
  (a part with a `foot` connector, like `pf.leg`'s shin). `forward` names them front/rear.
- `cmd_vel` (normalized, like the Gamepad) walks, strafes and turns; zero stands.
- `imu` (optional) holds the body level. `joint_states` (each leg Joint Controller's `state`) is how it
  knows a foot is down — the efforts give each foot's ground force, no foot sensor needed; `contact_*`
  sensors, where a robot has them, override that per leg.
- `joint_commands` goes to every leg joint's Joint Controller.

Tests: `PYTHONPATH=common:orchestrators/legged_gait python3 -m pytest orchestrators/legged_gait/test`
