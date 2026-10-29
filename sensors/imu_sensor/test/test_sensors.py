"""The three sensor kernels take the same `update_reading(data)` from the simulator and the robot.
Run from the repo root:
    PYTHONPATH=common:sensors/imu_sensor:sensors/contact_sensor:sensors/range_sensor python3 -m pytest sensors/imu_sensor/test
"""
import pytest

from imu_sensor.kernel import ImuKernel
from contact_sensor.kernel import ContactSensorKernel
from range_sensor.kernel import RangeSensorKernel


def kernel(cls, **params):
    out = {}
    k = cls("s", {"sensor_id": "6ae3abc", **params}, on_emit=lambda pin, data: out.__setitem__(pin, data), on_log=lambda _m: None)
    k.on_emit = lambda pin, data: out.__setitem__(pin, data)
    return k, out


def test_imu_forwards_the_simulators_reading_as_sensor_msgs_imu():
    k, out = kernel(ImuKernel)
    assert k.sensor_id == "6ae3abc"
    k.update_reading({"orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                      "angular_velocity": {"x": 0.1, "y": 0.0, "z": 0.0},
                      "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 9.81}})
    assert out["imu"]["linear_acceleration"]["z"] == pytest.approx(9.81)
    assert out["imu"]["angular_velocity"]["x"] == pytest.approx(0.1)


def test_contact_takes_the_sims_in_contact_or_the_robots_bool():
    k, out = kernel(ContactSensorKernel)
    k.update_reading({"in_contact": True})
    assert out["contact"] == {"data": True}
    k.update_reading({"data": False})          # std_msgs/Bool from the adapter, as a dict
    assert out["contact"] == {"data": False}


def test_range_reports_metres_with_its_window():
    k, out = kernel(RangeSensorKernel)
    k.update_reading({"range": 0.42, "min_range": 0.04, "max_range": 4.0})
    r = out["range"]
    assert (r["range"], r["min_range"], r["max_range"]) == pytest.approx((0.42, 0.04, 4.0))
    assert r["radiation_type"] == 1
