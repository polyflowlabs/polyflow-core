import math
from collections import deque
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from common.polyflow_kernel import PolyflowKernel


LEGS = ("fl", "fr", "rl", "rr")
# The creep: each hind leg, then the fore leg on its side — the order that keeps the next support
# triangle under the body with the least shifting.
CRAWL_ORDER = ("rl", "fl", "rr", "fr")
_FORWARD = {"+X": (1.0, 0.0, 0.0), "-X": (-1.0, 0.0, 0.0), "+Z": (0.0, 0.0, 1.0), "-Z": (0.0, 0.0, -1.0)}
_UP = np.array([0.0, 1.0, 0.0])
# Studio's world is Y-up; an IMU reports its orientation in a Z-up world (+90° about X).
_YUP_TO_ZUP = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])


def _quat_matrix(q: Dict[str, Any]) -> np.ndarray:
    x, y, z, w = (float(q.get(k, d)) for k, d in (("x", 0.0), ("y", 0.0), ("z", 0.0), ("w", 1.0)))
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _pose_matrix(pose: Optional[Dict[str, Any]]) -> np.ndarray:
    """A serialized pose {pos: {x,y,z}, rot: {x,y,z,w}} (metres) → 4x4."""
    T = np.eye(4)
    if not pose:
        return T
    T[:3, :3] = _quat_matrix(pose.get("rot") or {})
    p = pose.get("pos") or {}
    T[:3, 3] = [float(p.get("x", 0.0)), float(p.get("y", 0.0)), float(p.get("z", 0.0))]
    return T


def _axis_angle(v: np.ndarray) -> np.ndarray:
    """Rotation by |v| about v (Rodrigues)."""
    a = float(np.linalg.norm(v))
    if a < 1e-12:
        return np.eye(3)
    k = v / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)


def _rot2(psi: float, v: np.ndarray) -> np.ndarray:
    c, s = math.cos(psi), math.sin(psi)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def _smooth(u: float) -> float:
    u = min(1.0, max(0.0, u))
    return u * u * (3.0 - 2.0 * u)


def _inside_triangle(p: np.ndarray, tri: List[np.ndarray]) -> float:
    """Signed distance from p to the nearest edge of the triangle — positive inside."""
    a, b, c = tri
    area = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    sign = 1.0 if area >= 0 else -1.0
    best = math.inf
    for u, v in ((a, b), (b, c), (c, a)):
        e = v - u
        n = np.array([-e[1], e[0]]) * sign / (np.linalg.norm(e) or 1.0)
        best = min(best, float(np.dot(p - u, n)))
    return best


class Leg:
    """One leg: the DOF joints from the base body to the body with the foot, and the foot's point on it.

    Forward kinematics compose exactly as the physics worker builds the articulation (the IK node's
    chain): child = parent · parentPose · Rx(q) · inv(childPose), the joint turning about its frame's X.
    Everything is in the BASE LINK's frame."""

    def __init__(self, name: str, chain: List[Dict[str, Any]], foot_in_tip: np.ndarray):
        self.name = name
        self.chain = chain
        self.foot_in_tip = foot_in_tip
        self.actuated = [i for i, j in enumerate(chain) if j["type"] in ("revolute", "continuous", "prismatic")]
        self.lower = np.array([chain[i]["lower"] for i in self.actuated])
        self.upper = np.array([chain[i]["upper"] for i in self.actuated])
        self.q = self._seed()

    def _seed(self) -> np.ndarray:
        """Off the straight-leg singularity: a straight knee can't move the foot up at all. Every joint
        starts at 0 (clamped into its range) and the last one — the knee — a little bent, the way its
        range allows."""
        q = np.clip(np.zeros(len(self.actuated)), self.lower, self.upper)
        if len(q):
            lo, hi = self.lower[-1], self.upper[-1]
            q[-1] = float(np.clip(0.15 if lo >= 0 else -0.15, lo, hi))
        return q

    def _full_q(self, q: np.ndarray) -> np.ndarray:
        full = np.zeros(len(self.chain))
        full[self.actuated] = q
        return full

    def fk(self, q: np.ndarray) -> Tuple[np.ndarray, List[Tuple[np.ndarray, np.ndarray, str]]]:
        """The foot's position, and each actuated joint's (origin, axis, type)."""
        T = np.eye(4)
        frames = []
        for joint, qi in zip(self.chain, self._full_q(q)):
            Tj = T @ joint["T_parent_pose"]
            if joint["type"] in ("revolute", "continuous", "prismatic"):
                frames.append((Tj[:3, 3].copy(), Tj[:3, 0].copy(), joint["type"]))
            M = np.eye(4)
            if joint["type"] in ("revolute", "continuous"):
                c, s = math.cos(qi), math.sin(qi)
                M[1:3, 1:3] = [[c, -s], [s, c]]
            elif joint["type"] == "prismatic":
                M[0, 3] = qi
            T = Tj @ M @ joint["T_child_pose_inv"]
        return (T @ self.foot_in_tip)[:3, 3], frames

    def foot(self, q: Optional[np.ndarray] = None) -> np.ndarray:
        return self.fk(self.q if q is None else q)[0]

    def solve(self, target: np.ndarray, iterations: int = 12, damping: float = 0.004, tol: float = 2e-4) -> float:
        """Damped least squares from the last solution — a few analytic-Jacobian steps a tick, cheap
        enough for four legs at the gait's rate (a scipy solve per leg per tick was not)."""
        q = self.q.copy()
        err = math.inf
        for _ in range(iterations):
            p, frames = self.fk(q)
            e = target - p
            err = float(np.linalg.norm(e))
            if err < tol:
                break
            J = np.column_stack([np.cross(ax, p - o) if t != "prismatic" else ax for o, ax, t in frames])
            dq = J.T @ np.linalg.solve(J @ J.T + damping ** 2 * np.eye(3), e)
            step = float(np.max(np.abs(dq))) if len(dq) else 0.0
            if step > 0.3:
                dq *= 0.3 / step
            q = np.clip(q + dq, self.lower, self.upper)
        self.q = q
        return float(np.linalg.norm(target - self.foot(q)))

    def joint_ids(self) -> List[str]:
        return [self.chain[i]["_id"] for i in self.actuated]


class LeggedGaitKernel(PolyflowKernel):
    """
    A statically stable crawl for a four-legged robot.

    Finds the legs itself from the injected `flat_articulation`: each leg is the chain of joints from
    the base body to a body carrying a FOOT (a part with a `foot` connector — pf.leg's shin). The four
    are named front/rear × left/right from where their feet sit, `forward` saying which way is front.

    Plans in a ground frame: each foot has a foothold there, and the body a position and heading.
    A crawl cycle is four quarters, one per leg (hind-left, fore-left, hind-right, fore-right):
      1. SHIFT: with all four feet down, move the body so its centre of mass sits over the centroid of
         the three feet that will stay down — the support triangle — before anything lifts.
      2. SWING: lift that leg (`step_height`), carry it to its next foothold half a stride ahead of the
         hip, and put it down. With foot contact wired it keeps lowering until the foot touches
         (up to `probe_depth`), or stops early when it touches sooner.
    Feet in stance never move in the ground frame, so the legs never fight each other: the body moves
    by shifting over the feet, and that is the walk.

    Levels the body from the IMU: the body's tilt (calibrated against its first reading, taken
    standing as built) slowly rotates the foot targets the other way, so the body holds level on a
    slope and when a leg sags under load.

    Each leg's joint targets come from a damped-least-squares IK on the leg's chain, emitted together
    as one joint_commands message for the leg's Joint Controllers (each takes its own joint).

    Inputs:
        cmd_vel     Twist, normalized [-1, 1] like the Gamepad: linear.x forward, linear.y left,
                    angular.z turn left — scaled by max_speed and max_yaw_rate. Zero (or none for
                    cmd_timeout_s) stands still.
        imu         sensor_msgs/Imu from an IMU Sensor node on the body.
        contact_fl / contact_fr / contact_rl / contact_rr   std_msgs/Bool from each foot's Contact
                    Sensor node.
    Output:
        joint_commands  {name: [joint ids], positions: [rad | m]} for every leg joint.
    """

    _STRUCTURE_PARAMS = ("flat_articulation", "forward")

    def setup(self):
        self.rate_hz = float(self.get_param("rate_hz", 50.0))
        self.dt = 1.0 / max(self.rate_hz, 1e-3)
        self._read_tunables()

        self.flat: Dict[str, Any] = self.get_param("flat_articulation", {}) or {}
        self.legs: Dict[str, Leg] = {}
        self._build()

        self._t = 0.0
        self._cmd = np.zeros(3)
        self._cmd_t = -math.inf
        self._imu_ref: Optional[np.ndarray] = None
        self._imu_t = 0.0
        self._level = np.zeros(3)
        self._contact: Dict[str, bool] = {}
        self._state = "stand"
        self._quarter = 0
        self._s = 0.0
        self._stopped_quarters = 0
        self._plan: Optional[Dict[str, Any]] = None
        self._lift = {k: 0.0 for k in LEGS}
        self._z_off = {k: 0.0 for k in LEGS}
        self._b = np.zeros(2)
        self._psi = 0.0
        # The commanded body: it moves exactly as cmd_vel says, and footholds are placed off it. The real
        # body (`_b`, `_psi`) sways round it, shifting over each support triangle.
        self._b_v = np.zeros(2)
        self._psi_v = 0.0
        self.footholds = {k: self._neutral[k].copy() for k in self.legs}

    def _read_tunables(self):
        g = lambda k, d: float(self.get_param(k, d))
        self.cycle_s = max(g("cycle_s", 4.0), 0.4)
        self.step_height = g("step_height", 0.04)
        self.crouch = min(max(g("crouch", 0.8), 0.3), 1.0)
        self.max_speed = g("max_speed", 0.03)
        self.max_yaw_rate = g("max_yaw_rate", 0.25)
        self.max_stride = g("max_stride", 0.12)
        self.shift_fraction = min(max(g("shift_fraction", 0.35), 0.05), 0.9)
        self.shift_gain = min(max(g("shift_gain", 1.0), 0.0), 1.0)
        self.com_offset = np.array([g("com_offset_x", 0.0), g("com_offset_y", 0.0)])
        self.level_gain = g("level_gain", 1.5)
        self.max_level = math.radians(g("max_level_deg", 12.0))
        self.startup_s = max(g("startup_s", 2.0), 0.0)
        self.cmd_timeout_s = g("cmd_timeout_s", 1.0)
        self.probe_depth = g("probe_depth", 0.02)
        self.probe_speed = g("probe_speed", 0.05)

    def set_param(self, key: str, value: Any):
        """A tuning change applies in place — the gait keeps its footholds. Only a change to the robot
        (or which way is forward) rebuilds it; an empty push of the injected structure is ignored."""
        if key == "flat_articulation" and not value and self.legs:
            return
        old = self.parameters.get(key)
        self.parameters[key] = value
        if key in self._STRUCTURE_PARAMS or key == "rate_hz":
            if old != value:
                self.setup()
        else:
            self._read_tunables()

    # ─── Structure ───

    def _build(self):
        self._neutral: Dict[str, np.ndarray] = {}
        links = self.flat.get("links") or []
        joints = self.flat.get("joints") or []
        base = next((l for l in links if l.get("base")), None)
        if not base:
            if links:
                self.log("No base body on this robot — nothing to walk from")
            return

        # The body frame: the base link's, with the world's up and the chosen forward carried into it.
        R_base = _pose_matrix(base.get("restWorld"))[:3, :3]
        self._R_base = R_base
        self.up = R_base.T @ _UP
        fwd = R_base.T @ np.array(_FORWARD.get(str(self.get_param("forward", "+X")), _FORWARD["+X"]))
        fwd = fwd - np.dot(fwd, self.up) * self.up
        if np.linalg.norm(fwd) < 1e-6:
            self.log("`forward` points straight up on this body — pick another axis")
            return
        self.fwd = fwd / np.linalg.norm(fwd)
        self.left = np.cross(self.up, self.fwd)

        # Feet: every member with a `foot` off the base body. Without any, each leaf body's origin.
        # One foot per body: a foot-contact switch on a shin's foot carries a `foot` of its own (the face
        # it mates up with), so a body can list two — the lowest at rest is where it meets the ground.
        feet: List[Tuple[str, np.ndarray]] = []
        for link in links:
            if link is base:
                continue
            own = [_pose_matrix(m["foot"]) for m in link.get("members") or [] if m.get("foot")]
            if own:
                rest = _pose_matrix(link.get("restWorld"))
                feet.append((str(link.get("id")), min(own, key=lambda T: float((rest @ T)[1, 3]))))
        if not feet:
            parents = {str(j.get("parentLink")) for j in joints}
            feet = [(str(l.get("id")), np.eye(4)) for l in links if l is not base and str(l.get("id")) not in parents]
            if feet:
                self.log("No part has a `foot` connector — using each leaf body's origin as its foot")

        by_parent: Dict[str, List[Dict[str, Any]]] = {}
        for j in joints:
            by_parent.setdefault(str(j.get("parentLink", "")), []).append(j)
        legs = []
        for link_id, foot in feet:
            path = self._path(str(base.get("id")), link_id, by_parent)
            chain = [self._joint(j) for j in path or []]
            if any(j["type"] != "fixed" for j in chain):
                legs.append(Leg("", chain, foot))
        if len(legs) != 4:
            self.log(f"Found {len(legs)} legs with feet; the crawl needs exactly 4")
            return

        # Name them by where the feet sit: front/rear along forward, left/right along left.
        rest = [lg.foot(np.zeros(len(lg.actuated))) for lg in legs]
        cx = np.mean([np.dot(p, self.fwd) for p in rest])
        cy = np.mean([np.dot(p, self.left) for p in rest])
        for lg, p in zip(legs, rest):
            lg.name = ("f" if np.dot(p, self.fwd) > cx else "r") + ("l" if np.dot(p, self.left) > cy else "r")
        names = sorted(lg.name for lg in legs)
        if names != sorted(LEGS):
            self.log(f"The feet don't sit at four corners (got {names}) — is `forward` right?")
            return
        self.legs = {lg.name: lg for lg in legs}

        # Standing: each foot under where it hangs, raised so the legs bend to `crouch` of their length.
        heights = []
        for name, lg in self.legs.items():
            p = lg.foot(np.zeros(len(lg.actuated)))
            _, frames = lg.fk(np.zeros(len(lg.actuated)))
            reach = max(float(np.dot(o - p, self.up)) for o, _, _ in frames)
            heights.append(reach)
            self._neutral[name] = np.array([np.dot(p, self.fwd), np.dot(p, self.left)])
        self._rest_h = {n: float(np.dot(lg.foot(np.zeros(len(lg.actuated))), self.up)) for n, lg in self.legs.items()}
        self._raise = (1.0 - self.crouch) * float(np.mean(heights))
        self._com0 = np.mean(list(self._neutral.values()), axis=0) + self.com_offset
        self.log(f"Legged gait: legs {', '.join(f'{n}={len(lg.actuated)} joints' for n, lg in sorted(self.legs.items()))}; "
                 f"stands {self._raise * 1000:.0f} mm crouched")

    @staticmethod
    def _path(start: str, goal: str, by_parent: Dict[str, List[Dict[str, Any]]]) -> Optional[List[Dict[str, Any]]]:
        queue: deque = deque([(start, [])])
        seen = {start}
        while queue:
            lid, path = queue.popleft()
            if lid == goal:
                return path
            for j in by_parent.get(lid, []):
                child = str(j.get("childLink", ""))
                if child and child not in seen:
                    seen.add(child)
                    queue.append((child, path + [j]))
        return None

    @staticmethod
    def _joint(j: Dict[str, Any]) -> Dict[str, Any]:
        jt = str(j.get("jointType", "fixed")).lower()
        specs = j.get("specs") or {}
        lo = (specs.get("lower_position") or {}).get("value")
        hi = (specs.get("upper_position") or {}).get("value")
        if jt == "continuous" or (lo is None and hi is None):
            lo, hi = -math.pi, math.pi
        return {
            "_id": str(j.get("id", "")),
            "type": jt,
            "lower": float(lo if lo is not None else -math.pi),
            "upper": float(hi if hi is not None else math.pi),
            "T_parent_pose": _pose_matrix(j.get("parentPose")),
            "T_child_pose_inv": np.linalg.inv(_pose_matrix(j.get("childPose"))),
        }

    # ─── Inputs ───

    def process_input(self, pin_id: str, data: dict):
        if not self.should_run(trigger_info={"pin_id": pin_id}):
            return
        if pin_id == "cmd_vel":
            lin, ang = (data or {}).get("linear") or {}, (data or {}).get("angular") or {}
            clamp = lambda v: max(-1.0, min(1.0, float(v or 0.0)))
            self._cmd = np.array([clamp(lin.get("x")), clamp(lin.get("y")), clamp(ang.get("z"))])
            self._cmd_t = self._t
        elif pin_id == "imu":
            self._on_imu((data or {}).get("orientation") or {})
        elif pin_id.startswith("contact_") and pin_id[8:] in LEGS:
            value = (data or {}).get("data", data) if isinstance(data, dict) else data
            self._contact[pin_id[8:]] = bool(value)

    def _on_imu(self, orientation: Dict[str, Any]):
        if not self.legs:
            return
        R = _quat_matrix(orientation)
        if self._imu_ref is None:
            # Sensor → body, from the first reading: the body is standing as it was built then.
            self._imu_ref = self._R_base.T @ _YUP_TO_ZUP.T @ R
            self._imu_t = self._t
            return
        dt = min(self._t - self._imu_t, 0.1)
        self._imu_t = self._t
        up_now = self._imu_ref @ R.T @ np.array([0.0, 0.0, 1.0])
        # The world's up, seen from the body, leans toward the back when the nose drops. The correction
        # turns the feet back the other way: front feet reach further down, rear ones less.
        tilt = np.cross(up_now, self.up)
        self._level = self._level + self.level_gain * dt * tilt
        n = float(np.linalg.norm(self._level))
        if n > self.max_level:
            self._level *= self.max_level / n

    # ─── The gait ───

    def poll(self):
        if not self.should_run(trigger_info={"pin_id": "__tick__"}):
            return
        if self.legs:
            self.step(self.dt)
            self._emit_commands()

    def _command(self) -> np.ndarray:
        if self._t - self._cmd_t > self.cmd_timeout_s:
            return np.zeros(3)
        return self._cmd * np.array([self.max_speed, self.max_speed, self.max_yaw_rate])

    def com(self) -> np.ndarray:
        """The body's centre of mass in the ground frame."""
        return self._b + _rot2(self._psi, self._com0)

    def _support(self, swing: str) -> List[np.ndarray]:
        return [self.footholds[k] for k in LEGS if k != swing]

    def _plan_quarter(self):
        leg = CRAWL_ORDER[self._quarter]
        v = self._command()
        stride = v[:2] * self.cycle_s
        n = float(np.linalg.norm(stride))
        if n > self.max_stride:
            stride *= self.max_stride / n
        # Where the commanded body will be when this leg touches down, a quarter from now.
        quarter_s = self.cycle_s / 4.0
        psi1 = self._psi_v + v[2] * quarter_s
        b_td = self._b_v + _rot2(self._psi_v, v[:2] * quarter_s)
        # Body: its COM over the centroid of the three feet that stay down.
        centroid = np.mean(self._support(leg), axis=0)
        neutral_b = np.mean(list(self.footholds.values()), axis=0) - _rot2(psi1, self._com0)
        b1 = neutral_b + self.shift_gain * ((centroid - _rot2(psi1, self._com0)) - neutral_b)
        # Foothold: half a stride ahead of where its hip will be on the COMMANDED body at touchdown — not
        # the body shifted over a triangle, whose offset would leave the feet splayed when it stops.
        place = b_td + _rot2(psi1, self._neutral[leg] + stride / 2.0)
        self._plan = {"leg": leg, "b0": self._b.copy(), "b1": b1, "psi0": self._psi, "psi1": psi1,
                      "lift": self.footholds[leg].copy(), "place": place, "probing": False,
                      "still": not bool(np.any(np.abs(v) > 1e-6))}

    def step(self, dt: float):
        self._t += dt
        moving = bool(np.any(np.abs(self._command()) > 1e-6))
        ready = self._t >= self.startup_s

        if self._state == "stand":
            # Settle the body over all four feet.
            target = np.mean(list(self.footholds.values()), axis=0) - _rot2(self._psi, self._com0)
            self._b += (target - self._b) * min(1.0, dt / 0.3)
            if moving and ready:
                self._state, self._quarter, self._s, self._stopped_quarters = "walk", 0, 0.0, 0
                self._b_v, self._psi_v = target.copy(), self._psi
                self._plan_quarter()
            return

        v = self._command()
        self._b_v = self._b_v + _rot2(self._psi_v, v[:2] * dt)
        self._psi_v += v[2] * dt

        p = self._plan
        quarter_s = self.cycle_s / 4.0
        if not p["probing"]:
            self._s += dt / quarter_s
        s = self._s
        leg = p["leg"]
        if s < self.shift_fraction:
            u = _smooth(s / self.shift_fraction)
            self._b = p["b0"] + (p["b1"] - p["b0"]) * u
            self._psi = p["psi0"] + (p["psi1"] - p["psi0"]) * u
            return
        self._b, self._psi = p["b1"].copy(), p["psi1"]

        u = min(1.0, (s - self.shift_fraction) / (1.0 - self.shift_fraction))
        touching = self._contact.get(leg)
        if u < 1.0:
            self._z_off[leg] = 0.0
            self.footholds[leg] = p["lift"] + (p["place"] - p["lift"]) * _smooth(u)
            self._lift[leg] = self.step_height * math.sin(math.pi * u)
            # Touched down early on the way down: stay there.
            if u > 0.5 and touching:
                self._z_off[leg] = self._lift[leg]
                self._lift[leg] = 0.0
                self._finish_quarter(moving)
            return

        self.footholds[leg] = p["place"].copy()
        self._lift[leg] = 0.0
        # Not down yet with a contact sensor on it: keep lowering it until it is, or it's out of reach.
        if touching is False and self._z_off[leg] > -self.probe_depth:
            p["probing"] = True
            self._z_off[leg] = max(-self.probe_depth, self._z_off[leg] - self.probe_speed * dt)
            return
        self._finish_quarter(moving)

    def _finish_quarter(self, moving: bool):
        # Only a step PLANNED standing still counts toward stopping: the one in flight when the command
        # stopped still carries its stride.
        self._stopped_quarters = self._stopped_quarters + 1 if self._plan["still"] and not moving else 0
        self._quarter = (self._quarter + 1) % 4
        self._s = 0.0
        # Stopped: one full cycle in place brings every foot back under its hip, then stand.
        if not moving and self._stopped_quarters >= 4:
            self._state, self._plan = "stand", None
            return
        self._plan_quarter()

    def foot_target(self, leg: str) -> np.ndarray:
        """Where the leg's foot goes now, in the base body's frame."""
        rel = _rot2(-self._psi, self.footholds[leg] - self._b)
        ramp = _smooth(self._t / self.startup_s) if self.startup_s > 0 else 1.0
        h = self._rest_h[leg] + ramp * self._raise + self._lift[leg] + self._z_off[leg]
        p = rel[0] * self.fwd + rel[1] * self.left + h * self.up
        return _axis_angle(self._level) @ p

    def _emit_commands(self):
        names: List[str] = []
        positions: List[float] = []
        for name in LEGS:
            lg = self.legs[name]
            lg.solve(self.foot_target(name))
            names += lg.joint_ids()
            positions += [float(v) for v in lg.q]
        self.emit("joint_commands", {"name": names, "positions": positions})
