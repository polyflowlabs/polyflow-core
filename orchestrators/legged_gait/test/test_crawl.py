"""The crawl on a synthetic quadruped. Run from the repo root:
    PYTHONPATH=common:orchestrators/legged_gait python3 -m pytest orchestrators/legged_gait/test
"""
import math

import numpy as np
import pytest

from legged_gait.kernel import CRAWL_ORDER, LEGS, LeggedGaitKernel, _inside_triangle, _rot2

THIGH = SHIN = 0.13
S45 = math.sqrt(0.5)
# A joint turns about its frame's X: abduction about forward (+X) as is; pitch and knee about the
# lateral axis (+Z), a frame turned −90° about Y.
LATERAL = {"x": 0.0, "y": -S45, "z": 0.0, "w": S45}
IDENT = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}


def pose(x=0.0, y=0.0, z=0.0, rot=IDENT):
    return {"pos": {"x": x, "y": y, "z": z}, "rot": dict(rot)}


def limits(lo, hi):
    return {"lower_position": {"value": lo}, "upper_position": {"value": hi}}


def quadruped():
    """Body 200 × 120 mm (X forward, Y up, Z right); each leg: abduction, hip pitch, knee, a foot."""
    links = [{"id": "body", "base": True, "restWorld": pose(), "members": [{"partInstanceId": "body", "poseInLink": pose()}]}]
    joints = []
    for name, (x, z) in {"fl": (0.1, -0.06), "fr": (0.1, 0.06), "rl": (-0.1, -0.06), "rr": (-0.1, 0.06)}.items():
        hip, thigh, shin = f"{name}_hip", f"{name}_thigh", f"{name}_shin"
        links += [
            {"id": hip, "base": False, "restWorld": pose(x, 0, z), "members": [{"partInstanceId": hip, "poseInLink": pose()}]},
            {"id": thigh, "base": False, "restWorld": pose(x, 0, z), "members": [{"partInstanceId": thigh, "poseInLink": pose()}]},
            {"id": shin, "base": False, "restWorld": pose(x, -THIGH, z),
             "members": [{"partInstanceId": shin, "poseInLink": pose(), "foot": pose(0, -SHIN, 0)}]},
        ]
        joints += [
            {"id": f"{name}_abd", "parentLink": "body", "childLink": hip, "jointType": "revolute",
             "specs": limits(-0.44, 0.44), "parentPose": pose(x, 0, z), "childPose": pose()},
            {"id": f"{name}_pitch", "parentLink": hip, "childLink": thigh, "jointType": "revolute",
             "specs": limits(-1.5, 1.5), "parentPose": pose(rot=LATERAL), "childPose": pose(rot=LATERAL)},
            {"id": f"{name}_knee", "parentLink": thigh, "childLink": shin, "jointType": "revolute",
             "specs": limits(-2.6, 0.0), "parentPose": pose(0, -THIGH, 0, LATERAL), "childPose": pose(rot=LATERAL)},
        ]
    return {"links": links, "joints": joints}


def kernel(**params):
    emitted = []
    k = LeggedGaitKernel("gait", {"flat_articulation": quadruped(), "startup_s": 0.5, **params},
                         on_emit=lambda pin, data: emitted.append((pin, data)), on_log=lambda _m: None)
    return k, emitted


def twist(x=0.0, y=0.0, z=0.0):
    return {"linear": {"x": x, "y": y, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": z}}


def run(k, seconds, each=None, cmd=None):
    for _ in range(int(round(seconds * k.rate_hz))):
        if cmd is not None:
            k.process_input("cmd_vel", cmd)
        k.poll()
        if each:
            each(k)


def test_finds_four_legs_and_names_them_by_corner():
    k, _ = kernel()
    assert sorted(k.legs) == sorted(LEGS)
    fl = k.legs["fl"].foot(np.zeros(3))
    assert fl[0] > 0 and fl[2] < 0          # front is +X, left is −Z (Y up, Z right)
    assert k.legs["fl"].joint_ids() == ["fl_abd", "fl_pitch", "fl_knee"]


def test_stands_crouched_with_every_foot_on_target():
    k, emitted = kernel()
    run(k, 1.0)
    for name, leg in k.legs.items():
        assert np.linalg.norm(leg.foot() - k.foot_target(name)) < 1e-3
        assert leg.foot()[1] == pytest.approx(-(THIGH + SHIN) * 0.8, abs=1e-3)
    pin, data = emitted[-1]
    assert pin == "joint_commands" and len(data["name"]) == 12 and len(data["positions"]) == 12
    # The knee bends the way its range allows.
    assert all(k.legs[n].q[2] < -0.5 for n in LEGS)


def test_walks_forward_with_the_com_always_over_the_support_triangle():
    k, _ = kernel()
    run(k, 1.0)
    start = k.com().copy()
    margins, errors = [], []

    def check(k):
        for name, leg in k.legs.items():
            errors.append(np.linalg.norm(leg.foot() - k.foot_target(name)))
        p = k._plan
        if k._state == "walk" and p and k._lift[p["leg"]] > 1e-4:
            margins.append(_inside_triangle(k.com(), k._support(p["leg"])))

    run(k, 3 * k.cycle_s, check, cmd=twist(x=1.0))
    run(k, 2 * k.cycle_s, cmd=twist())                            # stop, so the body is centred again
    travelled = k.com() - start
    assert travelled[0] == pytest.approx(3 * k.max_speed * k.cycle_s, abs=0.03)   # the commanded 0.36 m
    assert abs(travelled[1]) < 0.005
    assert min(margins) > 0.01                                    # a centimetre inside, every swing tick (at the default half-way shift)
    assert max(errors) < 2e-3                                     # the IK keeps up


def test_steps_in_the_crawl_order():
    k, _ = kernel()
    run(k, 1.0)
    seen = []

    def watch(k):
        p = k._plan
        if p and k._lift[p["leg"]] > 1e-4 and (not seen or seen[-1] != p["leg"]):
            seen.append(p["leg"])

    run(k, k.cycle_s, watch, cmd=twist(x=1.0))
    assert tuple(seen) == CRAWL_ORDER


def test_stops_and_settles_back_under_the_hips():
    k, _ = kernel()
    run(k, 1.0)
    run(k, 2 * k.cycle_s, cmd=twist(x=1.0))
    run(k, 2 * k.cycle_s, cmd=twist())
    assert k._state == "stand"
    for name in LEGS:
        rel = _rot2(-k._psi, k.footholds[name] - k._b)
        assert np.linalg.norm(rel - k._neutral[name]) < 0.01


def test_turns_left_on_angular_z_with_no_hip_travelling_further_than_a_stride():
    k, _ = kernel()
    run(k, 1.0)
    k.process_input("cmd_vel", twist(z=1.0))
    w = k._command()[2]
    reach = max(abs(w) * np.linalg.norm(n) for n in k._neutral.values()) * k.cycle_s
    assert 0 < w <= k.max_yaw_rate and reach == pytest.approx(min(k.max_stride, k.max_yaw_rate * max(np.linalg.norm(n) for n in k._neutral.values()) * k.cycle_s))
    margins = []

    def check(k):
        p = k._plan
        if k._state == "walk" and p and k._lift[p["leg"]] > 1e-4:
            margins.append(_inside_triangle(k.com(), k._support(p["leg"])))

    run(k, 2 * k.cycle_s, check, cmd=twist(z=1.0))
    assert k._psi == pytest.approx(2 * k.cycle_s * w, rel=0.3)
    assert min(margins) > 0.01


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def test_levels_a_nose_down_body_by_reaching_further_with_the_front_feet():
    k, _ = kernel()
    to_zup = (S45, 0.0, 0.0, S45)                 # the IMU's world is Z-up: +90° about X
    nose_down = (0.0, 0.0, -math.sin(0.05), math.cos(0.05))   # −0.1 rad about Z (right) in the Y-up world
    level = dict(zip("xyzw", to_zup))
    tilted = dict(zip("xyzw", _qmul(to_zup, nose_down)))
    k.process_input("imu", {"orientation": tilted})   # a stale first reading: ignored before startup ends
    run(k, 1.0)
    k.process_input("imu", {"orientation": level})
    for _ in range(100):
        k.process_input("imu", {"orientation": tilted})
        k.poll()
    front = np.mean([k.foot_target(n)[1] for n in ("fl", "fr")])
    rear = np.mean([k.foot_target(n)[1] for n in ("rl", "rr")])
    assert front < rear - 0.005


def test_a_foot_that_finds_no_ground_reaches_down_for_it():
    k, _ = kernel()
    run(k, 1.0)
    k.process_input("contact_rl", {"data": False})
    reached = []

    def watch(k):
        reached.append(k._z_off["rl"])

    run(k, k.cycle_s / 2, watch, cmd=twist(x=1.0))
    assert min(reached) == pytest.approx(-k.probe_depth)
    assert k._plan["leg"] != "rl"                # it gave up at probe_depth and the crawl moved on


def test_a_foot_that_touches_early_stays_where_it_touched():
    k, _ = kernel()
    run(k, 1.0)
    k.process_input("contact_rl", {"data": True})
    run(k, k.cycle_s / 4 * 0.9, cmd=twist(x=1.0))
    assert k._plan["leg"] == "fl"                # rl's quarter ended early
    assert k._z_off["rl"] > 0.0


def test_a_foot_switch_on_the_foot_is_still_one_leg():
    flat = quadruped()
    for link in flat["links"]:
        if link["id"].endswith("_shin"):
            # The switch mates up onto the shin's foot with its own `foot` face; its sole is 8 mm lower.
            link["members"].append({"partInstanceId": link["id"] + "_switch", "poseInLink": pose(0, -SHIN, 0), "foot": pose(0, -SHIN, 0)})
            link["members"].append({"partInstanceId": link["id"] + "_sole", "poseInLink": pose(0, -SHIN - 0.008, 0), "foot": pose(0, -SHIN - 0.008, 0)})
    k = LeggedGaitKernel("gait", {"flat_articulation": flat}, on_emit=lambda *_: None, on_log=lambda _m: None)
    assert sorted(k.legs) == sorted(LEGS)
    assert k.legs["fl"].foot(np.zeros(3))[1] == pytest.approx(-THIGH - SHIN - 0.008)


def test_a_robot_without_four_feet_stays_idle():
    flat = quadruped()
    flat["links"] = [l for l in flat["links"] if not l["id"].startswith("rr")]
    flat["joints"] = [j for j in flat["joints"] if not j["id"].startswith("rr")]
    logs = []
    k = LeggedGaitKernel("gait", {"flat_articulation": flat}, on_emit=lambda *_: None, on_log=logs.append)
    k.poll()
    assert not k.legs and any("needs exactly 4" in m for m in logs)


def _feed_efforts(k, leg, f_up, q=None):
    """Joint states for one leg whose foot the ground pushes up with `f_up` N (τ = Jᵀ·(−F))."""
    lg = k.legs[leg]
    q = lg.q if q is None else q
    p, frames = lg.fk(q)
    J = np.column_stack([np.cross(ax, p - o) for o, ax, _ in frames])
    tau = J.T @ (-f_up * k.up)
    k.process_input("joint_states", {"name": lg.joint_ids(), "position": [float(v) for v in q], "effort": [float(v) for v in tau]})


def test_reads_each_foots_ground_force_from_its_joint_efforts():
    k, _ = kernel()
    run(k, 1.0)
    _feed_efforts(k, "fl", 20.0)
    assert k.foot_force("fl") == pytest.approx(20.0, rel=1e-6)
    assert k.foot_force("fr") is None              # not reported yet


def test_knows_a_foot_is_down_from_efforts_alone():
    k, _ = kernel()
    for leg in LEGS:
        _feed_efforts(k, leg, 15.0)                # standing: 60 N on four feet
    run(k, 1.0)
    assert k._weight_n == pytest.approx(60.0, rel=0.05)
    assert all(k.touching(leg) for leg in LEGS)
    _feed_efforts(k, "rl", 0.5)                    # lifted
    for _ in range(10):
        k.poll()
    assert k.touching("rl") is False and k.touching("fl") is True


def test_a_foot_with_no_force_on_it_reaches_down_for_the_ground():
    k, _ = kernel()
    for leg in LEGS:
        _feed_efforts(k, leg, 15.0)
    run(k, 1.0)
    reached = []

    def unloaded_rl(k):
        _feed_efforts(k, "rl", 0.0)                # rl never meets the ground
        reached.append(k._z_off["rl"])

    run(k, k.cycle_s / 2, unloaded_rl, cmd=twist(x=1.0))
    assert min(reached) == pytest.approx(-k.probe_depth)


def test_a_contact_sensor_overrides_the_effort_estimate():
    k, _ = kernel()
    for leg in LEGS:
        _feed_efforts(k, leg, 15.0)
    run(k, 1.0)
    k.process_input("contact_fl", {"data": False})
    assert k.touching("fl") is False


def test_holds_a_single_walk_command_and_stops_on_zero():
    k, _ = kernel()
    k.process_input("cmd_vel", twist(x=1.0))         # sent once, as a sim test or a mission does
    run(k, 1.0 + k.cycle_s)
    assert k._state == "walk"
    k.process_input("cmd_vel", twist())
    run(k, 2 * k.cycle_s)
    assert k._state == "stand"


def test_a_crouch_change_moves_the_standing_height_at_once():
    k, _ = kernel()
    run(k, 1.0)
    k.set_param("crouch", 0.9)
    run(k, 0.5)
    assert k.legs["fl"].foot()[1] == pytest.approx(-(THIGH + SHIN) * 0.9, abs=2e-3)
