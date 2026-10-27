"""The IK kernel on a parts-built robot: the chain comes from the flat articulation Studio injects.

A planar two-link arm, joint axes along world Z: base → upper (0.2 m) → fore, with a tool part welded
0.15 m out on the fore body. Run from the repo root:
    PYTHONPATH=common:orchestrators/inverse_kinematics python3 -m pytest orchestrators/inverse_kinematics/test
"""
import math

import numpy as np

from inverse_kinematics.kernel import InverseKinematicsKernel

# A joint frame whose X (the twist axis) is world +Z: −90° about Y.
X_TO_Z = {"x": 0.0, "y": -math.sin(math.pi / 4), "z": 0.0, "w": math.cos(math.pi / 4)}
IDENTITY = {"pos": {"x": 0, "y": 0, "z": 0}, "rot": {"x": 0, "y": 0, "z": 0, "w": 1}}


def flat_arm():
    return {
        "links": [
            {"id": "base", "base": True, "members": [{"partInstanceId": "base", "poseInLink": IDENTITY}]},
            {"id": "upper", "members": [{"partInstanceId": "upper", "poseInLink": IDENTITY},
                                        {"partInstanceId": "upper_cover", "poseInLink": IDENTITY}]},
            {"id": "fore", "members": [{"partInstanceId": "fore", "poseInLink": IDENTITY},
                                       {"partInstanceId": "tool", "poseInLink": {"pos": {"x": 0.15, "y": 0, "z": 0}, "rot": IDENTITY["rot"]}}]},
        ],
        "joints": [
            {"id": "j_shoulder", "name": "Shoulder", "parentLink": "base", "childLink": "upper", "jointType": "revolute",
             "specs": {"lower_position": {"value": -math.pi}, "upper_position": {"value": math.pi}},
             "parentPose": {"pos": {"x": 0, "y": 0, "z": 0}, "rot": X_TO_Z}, "childPose": {"pos": {"x": 0, "y": 0, "z": 0}, "rot": X_TO_Z}},
            {"id": "j_elbow", "name": "Elbow", "parentLink": "upper", "childLink": "fore", "jointType": "revolute",
             "specs": {"lower_position": {"value": -math.pi}, "upper_position": {"value": math.pi}},
             "parentPose": {"pos": {"x": 0.2, "y": 0, "z": 0}, "rot": X_TO_Z}, "childPose": {"pos": {"x": 0, "y": 0, "z": 0}, "rot": X_TO_Z}},
        ],
        "unresolved": [],
    }


def kernel(**params):
    emitted, logs = [], []
    k = InverseKinematicsKernel("ik", {"flat_articulation": flat_arm(), **params},
                                on_emit=lambda pin, data: emitted.append((pin, data)), on_log=logs.append)
    k.on_emit = lambda pin, data: emitted.append((pin, data))
    k.on_log = logs.append
    return k, emitted, logs


def test_builds_the_chain_between_the_parts_bodies_and_puts_the_tool_point_on_the_part():
    k, _, _ = kernel(root_part_id="base", end_effector_part_id="tool")
    assert [j["_id"] for j in k._chain] == ["j_shoulder", "j_elbow"]
    np.testing.assert_allclose(k._ee_position(np.zeros(2)), [0.35, 0, 0], atol=1e-9)
    # Shoulder at 90°: the whole arm points along +Y.
    np.testing.assert_allclose(k._ee_position(np.array([math.pi / 2, 0])), [0, 0.35, 0], atol=1e-9)
    # Any member part names its body: the cover is on the upper arm's body.
    k2, _, _ = kernel(root_part_id="base", end_effector_part_id="upper_cover")
    assert [j["_id"] for j in k2._chain] == ["j_shoulder"]


def test_solves_a_target_and_commands_the_robot_joints_by_id():
    k, emitted, _ = kernel(root_part_id="base", end_effector_part_id="tool")
    k.process_input("target_pose", {"position": {"x": 0.2, "y": 0.15, "z": 0.0}})
    pin, cmd = emitted[-1]
    assert pin == "joint_commands" and cmd["name"] == ["j_shoulder", "j_elbow"]
    np.testing.assert_allclose(k._ee_position(np.array(cmd["positions"])), [0.2, 0.15, 0.0], atol=1e-3)


def test_accepts_part_ids_in_the_older_component_params():
    # IK nodes already on robots carry the old template: no root_part_id param, but a part id set where
    # the component used to go still resolves.
    k, _, _ = kernel(root_component_id="base", end_effector_component_id="tool")
    assert len(k._chain) == 2


def test_says_why_when_the_chain_cannot_be_built():
    _, _, logs = kernel(root_part_id="base", end_effector_part_id="nope")
    assert any("End-effector part 'nope' is not on this robot" in m for m in logs)
    _, _, logs = kernel(root_part_id="tool", end_effector_part_id="base")
    assert any("No path from the root part's body" in m for m in logs)


def test_live_updates_rebuild_the_chain_only_when_it_changes():
    # Studio pushes every parameter when a sim attaches; each push used to re-run setup().
    k, _, _ = kernel(root_part_id="base", end_effector_part_id="tool")
    builds = []
    original = k._build_flat_chain
    k._build_flat_chain = lambda: builds.append(1) or original()
    for key, value in [("root_part_id", "base"), ("end_effector_part_id", "tool"), ("components", []), ("joints", []),
                       ("flat_articulation", None), ("max_iterations", 50), ("tolerance", 0.0005)]:
        k.set_param(key, value)
    assert builds == []                          # nothing that defines the chain changed
    assert len(k._chain) == 2                    # an empty structure push didn't wipe it
    assert (k.max_iterations, k.tolerance) == (50, 0.0005)
    k.set_param("end_effector_part_id", "upper_cover")
    assert builds == [1] and [j["_id"] for j in k._chain] == ["j_shoulder"]
