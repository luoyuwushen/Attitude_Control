"""待辨识模型上的控制假设验证，不替代上板试验。"""
import importlib.util
from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

import numpy as np

spec = importlib.util.spec_from_file_location("control_model", Path(__file__).parents[1] / "tools/control_model.py")
model = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = model
spec.loader.exec_module(model)


class ControlModelTests(unittest.TestCase):
    def test_quantized_discrete_gain_is_stable(self):
        _, gain, poles = model.design_gain()
        self.assertEqual(gain.shape, (4,))
        self.assertLess(max(abs(poles)), 1)

    def test_unforced_hanging_equilibrium(self):
        self.assertLess(np.linalg.norm(model.dynamics(np.array([math.pi, 0, 0, 0]), 0, model.Parameters())), 1e-10)

    def test_positive_voltage_sign(self):
        acceleration = model.dynamics(np.zeros(4), 1, model.Parameters())
        self.assertGreater(acceleration[1], 0)
        self.assertGreater(acceleration[3], 0)

    def test_from_hanging_and_hold(self):
        trace, control = model.simulate(12)
        self.assertGreaterEqual(control.captures, 1)
        self.assertEqual(control.falls, 0)
        self.assertFalse(control.sensor_fault)
        self.assertFalse(control.protection_fault)
        self.assertTrue(np.all(trace[-2000:, 6] == 1))
        self.assertLess(max(abs(trace[-2000:, 1])), math.radians(5))

    def test_small_external_torque_recovery(self):
        push = lambda t: 0.01 if 2 <= t < 2.05 else 0
        trace, control = model.simulate(5, initial=[0.03, 0, 0, 0], disturbance=push)
        self.assertEqual(control.falls, 0)
        self.assertLess(max(abs(trace[-1000:, 1])), math.radians(3))
        self.assertLess(max(abs(trace[:, 5])), 12.000001)

    def test_nonlinear_unforced_total_energy_conservation(self):
        p = replace(model.Parameters(), arm_damping=0, pendulum_damping=0, stall_torque=0)
        x = np.array([1.1, 2.0, 0.2, 0.8])
        initial = model.total_energy(x, p)
        for _ in range(1000):
            x = model.integrate(x, 0, p)
        self.assertLess(abs(model.total_energy(x, p) - initial), 1e-8)

    def test_fixed_point_pump_direction_and_energy_extraction(self):
        self.assertGreater(model.swing_q10([3217, -1024, 0, 0]), 0)
        self.assertLess(model.swing_q10([3217, 1024, 0, 0]), 0)
        # 底部20rad/s动能超过直立能量时泵反向，必须能够抽取能量。
        self.assertLess(model.swing_q10([3217, -20480, 0, 0]), 0)
        self.assertEqual(model.swing_q10([3217, 0, 0, 0], kick=True), 667)

    def test_unfavorable_sensor_installation_stops_at_blind_gap(self):
        p = replace(model.Parameters(), sensor_phase_down_degrees=85, theta_sign=1)
        trace, control = model.simulate(2, parameters=p)
        self.assertTrue(control.sensor_fault)
        self.assertEqual(trace[-1, 5], 0)


if __name__ == "__main__":
    unittest.main()
