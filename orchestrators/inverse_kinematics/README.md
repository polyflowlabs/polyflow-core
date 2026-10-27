# Inverse Kinematics

Solves joint positions for a target end-effector pose. Builds a kinematic chain from the configured root component to the end-effector component using the Robot's joint geometry, then runs a damped-least-squares solver (SciPy `least_squares`) on each new target. Outputs a per-joint trajectory point for the downstream Joint Controllers and republishes the achieved pose for monitoring or feedback loops.

The forward-kinematics chain matches the physics worker's convention exactly (`child = parent · parentPose · joint_rot(q) · inv(childPose)`), so simulated and on-robot results agree.

## Pins

| Pin | Direction | Type | Description |
| --- | --- | --- | --- |
| `target_pose` | subscribe | `geometry_msgs/Pose` | Desired end-effector pose. Bound to the Studio IK gizmo by default. |
| `joint_states` | subscribe | `sensor_msgs/JointState[]` | Fan-in of per-joint feedback; used to seed the next solve. |
| `joint_commands` | publish | `trajectory_msgs/JointTrajectoryPoint[]` | Solved joint positions for downstream controllers. |
| `current_pose` | publish | `geometry_msgs/Pose` | FK of the most recent solved joint state. Useful as a `seed_pose` source for Twist To Pose. |

## Parameters

- **Root Part** (`root_part_id`) — parts-built robots: any part on the chain's base body (usually the base part). Targets are in this part's frame.
- **End Effector Part** (`end_effector_part_id`) — parts-built robots: the part whose frame is the tool point (a gripper palm, a foot). Any part welded to the tip body works.
- **Robot Bodies** (`flat_articulation`) — auto-injected for a parts-built robot: its rigid bodies and the DOF joints between them, the same articulation the physics worker builds. Hidden in the UI. When present, the chain runs between the root part's body and the end-effector part's body, and `joint_commands` are keyed by the robot joint `_id` (what a Joint Controller on such a robot is bound to).
- **Root Component** / **End Effector** (`root_component_id`, `end_effector_component_id`) — legacy, component-built robots: the chain runs between these components. On a parts-built robot a part id here is accepted too, so IK nodes placed before the part params existed keep working.
- **Robot Components / Robot Joints** (`components`, `joints`) — auto-injected from the Robot on deployment (component-built robots). Hidden in the UI.
- **Max Solver Iterations** (`max_iterations`) — iteration cap for `least_squares`. Default `100`.
- **Position Tolerance** (`tolerance`) — in meters, default `0.001`. When the regularized solve (which keeps successive solutions on one branch) leaves the tip further than this from the target, a second pass with a far weaker pull closes the gap.
- **Damping Factor** (`damping`) — damping (λ) for the DLS solver. Higher values improve stability near singularities at the cost of step size. Default `0.01`.

## Typical use

Drop into a graph with one Joint Controller per actuated joint. Wire `joint_commands` to the trajectory inputs (each Joint Controller picks its own joint by name), and fan the Joint Controller `state` outputs back into `joint_states`. For teleop, route `current_pose` → Twist To Pose `seed_pose`, and `Twist To Pose.target_pose` → `target_pose` here — the operator drives the EE with a stick and IK keeps the joints consistent.

The solver dedups targets within ~0.1 mm to avoid re-solving when an upstream node (e.g. Twist To Pose) re-emits an unchanged target every tick.
