"""The mecanum mix. Run from the repo root:
    PYTHONPATH=common:orchestrators/mecanum_drive python3 -m pytest orchestrators/mecanum_drive/test
"""
import pytest

from mecanum_drive.kernel import MecanumDriveKernel


def kernel():
    emitted = {}
    k = MecanumDriveKernel("mecanum", {}, on_emit=lambda pin, data: emitted.__setitem__(pin, data["data"]), on_log=lambda _m: None)
    k.on_emit = lambda pin, data: emitted.__setitem__(pin, data["data"])
    return k, emitted


def twist(x=0.0, y=0.0, z=0.0):
    return {"linear": {"x": x, "y": y, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": z}}


def wheels(e):
    return [e["front_left_motor"], e["front_right_motor"], e["back_left_motor"], e["back_right_motor"]]


def test_forward_drives_every_wheel_forward():
    k, e = kernel()
    k.process_input("cmd_vel_automated", twist(x=0.5))
    assert wheels(e) == pytest.approx([0.5, 0.5, 0.5, 0.5])


def test_strafe_left_runs_the_diagonals_against_each_other():
    k, e = kernel()
    k.process_input("cmd_vel_automated", twist(y=0.5))
    assert wheels(e) == pytest.approx([-0.5, 0.5, 0.5, -0.5])


def test_turn_left_runs_the_sides_against_each_other():
    k, e = kernel()
    k.process_input("cmd_vel_automated", twist(z=0.5))
    assert wheels(e) == pytest.approx([-0.5, 0.5, -0.5, 0.5])


def test_a_saturating_command_scales_all_four_and_keeps_the_direction():
    k, e = kernel()
    k.process_input("cmd_vel_automated", twist(x=1.0, y=1.0))
    assert wheels(e) == pytest.approx([0.0, 1.0, 1.0, 0.0])
    k.process_input("cmd_vel_automated", twist(x=1.0, y=0.5, z=0.5))
    assert max(abs(w) for w in wheels(e)) == pytest.approx(1.0)
    assert wheels(e) == pytest.approx([0.0, 1.0, 0.5, 0.5])


def test_teleop_wins_while_fresh():
    k, e = kernel()
    k.process_input("cmd_vel_automated", twist(x=0.5))
    k.process_input("cmd_vel_teleop", twist(y=0.5))
    assert wheels(e) == pytest.approx([-0.5, 0.5, 0.5, -0.5])
    k.process_input("cmd_vel_automated", twist(x=0.5))
    assert wheels(e) == pytest.approx([-0.5, 0.5, 0.5, -0.5])


def test_a_partial_twist_reads_missing_axes_as_zero():
    k, e = kernel()
    k.process_input("cmd_vel_automated", {"linear": {"x": 0.3}})
    assert wheels(e) == pytest.approx([0.3, 0.3, 0.3, 0.3])
