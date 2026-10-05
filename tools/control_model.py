"""J280 非线性算法仿真；不代表实物验收或 FPGA RTL 闭环验证。

状态顺序为 [摆杆直立误差 rad, 摆杆速度 rad/s, 转臂位置 rad, 转臂速度 rad/s]。
本地手册确认 ADC 10 bit、电气行程 345°、编码器 1040 四倍频计数/圈。
杆长、质量、惯量、阻尼采用待辨识假设。电机采用官方标称堵转力矩，
并将标称减速后转速暂作空载转速来建立线性模型；手册未说明该转速的
负载条件。模型不包含静摩擦、启动死区、电感、齿隙、驱动限流和温升。
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math

import numpy as np
from scipy.linalg import expm, solve_discrete_are


@dataclass(frozen=True)
class Parameters:
    arm_length: float = 0.152  # m，借鉴开源模型，待实测
    arm_mass: float = 0.090  # kg，待实测
    pendulum_length: float = 0.150  # m，待实测
    pendulum_mass: float = 0.090  # kg，待实测
    arm_damping: float = 0.001  # N m s/rad，待辨识
    pendulum_damping: float = 0.0001  # N m s/rad，待辨识
    supply: float = 12.0  # V，本地介绍手册 2.2
    stall_torque: float = 6.6 * 0.0980665  # N m，6.6 kgf cm
    no_load_rpm: float = 549.0  # 假设：介绍手册2.2仅写减速后549±15 rpm，未标为空载
    dt: float = 0.001  # s，设计选择
    adc_codes: int = 1023
    adc_span_codes: float = 1023 * 3.3 / 5 / 2  # 前端 Vi/5+1 估计量程，非整10bit跨度
    adc_offset: float = 1023 / 2
    electrical_degrees: float = 345.0
    sensor_phase_down_degrees: float = 210.0  # 安装假设；85°反例会跨电气盲区并停机
    theta_sign: int = -1  # 示例安装方向配置；与顶层默认+1不同，需现场方向辨识
    encoder_cpr: int = 1040

    @property
    def jp(self):
        return self.pendulum_mass * self.pendulum_length**2 / 3

    @property
    def j0(self):
        return self.arm_mass * self.arm_length**2 / 3 + self.pendulum_mass * self.arm_length**2

    @property
    def coupling(self):
        return self.pendulum_mass * self.arm_length * self.pendulum_length / 2

    @property
    def gravity(self):
        return self.pendulum_mass * 9.81 * self.pendulum_length / 2

    @property
    def motor_k(self):
        return self.stall_torque / self.supply

    @property
    def motor_b(self):
        return self.stall_torque / (self.no_load_rpm * 2 * math.pi / 60)


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def feedback_q10(state_q10, gain_q10, catch_ms=512):
    """RTL反馈整数运算黄金值；4段刚度，乘积符号算术右移，结果permille。"""
    terms = np.asarray(state_q10, dtype=np.int64) * np.asarray(gain_q10, dtype=np.int64)
    arm_product = int(terms[2])
    if catch_ms < 128:
        arm_product >>= 2
    elif catch_ms < 256:
        arm_product >>= 1
    elif catch_ms < 384:
        arm_product = (arm_product >> 1) + (arm_product >> 2)
    total = int(terms[0] + terms[1] + terms[3]) + arm_product
    return int(np.clip(-(total >> 20), -1000, 1000))


def swing_q10(state_q10, kick=False):
    """RTL能量反馈整数运算黄金值；33点余弦表，与数学cos区别明确。"""
    theta, omega, arm, speed = map(int, state_q10)
    index = min(32, abs(theta) * 32 // 3217)
    cosine = round(math.cos(index * math.pi / 32) * 1024)
    deficit = 1024 - cosine - ((omega**2 * 167) >> 25)
    # 跟RTL符号位异或一致，包括cos=0及omega=0时的确定性定相。
    direction = -1 if (omega < 0) ^ (cosine < 0) else 1
    command = (667 if deficit >= 0 else -667) * direction
    command -= (25 * speed >> 10) + (42 * arm >> 10)
    return 667 if kick else int(np.clip(command, -800, 800))


def total_energy(x, p):
    theta, omega, _, speed = x
    return (0.5 * (p.j0 + p.jp * math.sin(theta)**2) * speed**2
            - p.coupling * math.cos(theta) * speed * omega
            + 0.5 * p.jp * omega**2 + p.gravity * math.cos(theta))


def dynamics(x: np.ndarray, voltage: float, p: Parameters, push: float = 0.0) -> np.ndarray:
    """欧拉拉格朗日模型，push 为作用在摆杆轴的外部力矩 N m。"""
    theta, omega, _, speed = x
    sn, cs = math.sin(theta), math.cos(theta)
    # 直立 θ=0，耦合惯量为负；正电压令 θ 的正向加速度增加。
    a, b, d = p.j0 + p.jp * sn**2, -p.coupling * cs, p.jp
    tau = p.motor_k * np.clip(voltage, -p.supply, p.supply)
    rhs_arm = tau - (p.arm_damping + p.motor_b) * speed
    rhs_arm -= 2 * p.jp * sn * cs * speed * omega + p.coupling * sn * omega**2
    rhs_pend = p.gravity * sn - p.pendulum_damping * omega + p.jp * sn * cs * speed**2 + push
    determinant = a * d - b * b
    arm_acc = (d * rhs_arm - b * rhs_pend) / determinant
    pend_acc = (a * rhs_pend - b * rhs_arm) / determinant
    return np.array([omega, pend_acc, speed, arm_acc])


def integrate(x, voltage, p, push=0.0):
    """1 ms 保持控制量，每 0.2 ms RK4 积分。"""
    step = p.dt / 5
    for _ in range(5):
        k1 = dynamics(x, voltage, p, push)
        k2 = dynamics(x + step * k1 / 2, voltage, p, push)
        k3 = dynamics(x + step * k2 / 2, voltage, p, push)
        k4 = dynamics(x + step * k3, voltage, p, push)
        x = x + step * (k1 + 2 * k2 + 2 * k3 + k4) / 6
    return x


def design_gain(p: Parameters = Parameters(), control_penalty: float = 0.5):
    """ZOH 离散 LQR；控制代价以电压为单位，输出另转 permille。"""
    origin = np.zeros(4)
    eps = 1e-6
    a = np.column_stack([(dynamics(origin + np.eye(4)[i] * eps, 0, p)
                           - dynamics(origin - np.eye(4)[i] * eps, 0, p)) / (2 * eps)
                          for i in range(4)])
    b = ((dynamics(origin, eps, p) - dynamics(origin, -eps, p)) / (2 * eps))[:, None]
    augmented = np.zeros((5, 5))
    augmented[:4, :4], augmented[:4, 4:] = a, b
    discrete = expm(augmented * p.dt)
    ad, bd = discrete[:4, :4], discrete[:4, 4:]
    if not math.isfinite(control_penalty) or control_penalty <= 0:
        raise ValueError('control_penalty must be finite and positive')
    q, r = np.diag([60.0, 4.0, 1.0, 0.2]), np.array([[control_penalty]])
    riccati = solve_discrete_are(ad, bd, q, r)
    gain = np.linalg.solve(r + bd.T @ riccati @ bd, bd.T @ riccati @ ad).ravel()
    gain_q10 = np.rint(gain * 1000 / p.supply * 1024).astype(np.int64)
    poles = np.linalg.eigvals(ad - bd @ (gain_q10[None, :] / 1024 * p.supply / 1000))
    return gain, gain_q10, poles


def design_handover_gain(p: Parameters = Parameters()):
    """H-only lower-bandwidth candidate; requires separate physical acceptance."""
    return design_gain(p, control_penalty=32.0)


def design_handover_lqi_gain(p: Parameters = Parameters(), integral_penalty: float = 1.0):
    """H-only five-state LQI candidate, ordered [theta,omega,arm,speed,int_arm].

    int_arm integrates arm error in rad*s. Returns voltage gains, all five
    permille gains in Q10, and ideal ZOH poles, like design_gain. The fifth
    Q10 coefficient describes the design only: production uses a separate
    Q24 accumulator with delta=arm_q10*237, ki=237/16.384 permille/(rad*s).
    Gates, saturation, derivative IIR and PWM timing require separate tests.
    """
    if not math.isfinite(integral_penalty) or integral_penalty <= 0:
        raise ValueError('integral_penalty must be finite and positive')
    origin = np.zeros(4)
    eps = 1e-6
    a = np.column_stack([(dynamics(origin + np.eye(4)[i] * eps, 0, p)
                         - dynamics(origin - np.eye(4)[i] * eps, 0, p)) / (2 * eps)
                         for i in range(4)])
    b = ((dynamics(origin, eps, p) - dynamics(origin, -eps, p)) / (2 * eps))[:, None]
    augmented = np.zeros((6, 6))
    augmented[:4, :4], augmented[:4, 5:] = a, b
    augmented[4, 2] = 1.0
    discrete = expm(augmented * p.dt)
    ad, bd = discrete[:5, :5], discrete[:5, 5:]
    q, r = np.diag([60.0, 4.0, 1.0, 0.2, integral_penalty]), np.array([[32.0]])
    riccati = solve_discrete_are(ad, bd, q, r)
    gain = np.linalg.solve(r + bd.T @ riccati @ bd, bd.T @ riccati @ ad).ravel()
    gain_q10 = np.rint(gain * 1000 / p.supply * 1024).astype(np.int64)
    poles = np.linalg.eigvals(ad - bd @ (gain_q10[None, :] / 1024 * p.supply / 1000))
    return gain, gain_q10, poles


class Controller:
    """Q10 传感器/反馈仿真，速度 IIR 1/8；能量泵用浮点作算法基准。"""
    def __init__(self, p: Parameters, quantized=True, initial_balance=False):
        self.p = p
        self.quantized = quantized
        self.gain, self.gain_q10, _ = design_gain(p)
        self.balance = initial_balance
        self.first = True
        self.last_theta = self.last_arm = 0.0
        self.omega = self.speed = 0.0
        self.q_omega = self.q_speed = 0
        self.age = 0
        self.catch_arm = 0.0
        self.captures = 0
        self.falls = 0
        self.last_true_theta = None
        self.sensor_phase_down = math.radians(p.sensor_phase_down_degrees)
        self.sensor_phase_up = (self.sensor_phase_down - math.pi) % (2 * math.pi)
        # 静止标定时16个相同整数ADC码求平均，无人为添加亚码精度。
        self.code_down = round(self._sensor_code(self.sensor_phase_down))
        self.code_up = round(self._sensor_code(self.sensor_phase_up))
        self.phase_direction = p.theta_sign * (-1 if self.code_up > self.code_down else 1)
        self.slope_q20 = 3294199 // (abs(self.code_up - self.code_down) * 16)
        self.sensor_fault = False
        self.protection_fault = False
        self.last_arm_count = 0

    def _sensor_code(self, phase):
        return self.p.adc_offset + phase / math.radians(self.p.electrical_degrees) * self.p.adc_span_codes

    def measured(self, x):
        theta, _, arm, _ = x
        arm_count = round(arm * self.p.encoder_cpr / (2 * math.pi))
        if self.quantized:
            # 电位器电气盲区采用无效采样保持，绝不把盲区伪装成真实12bit角度。
            # 选择跟两点标定符号一致的安装方向；仍需上板确认电机/编码器极性。
            phase = (self.sensor_phase_up + self.phase_direction * theta) % (2 * math.pi)
            if phase <= math.radians(self.p.electrical_degrees):
                # 近似 FPGA 1 ms 内16次ADC采样平均；运动可提供亚码精度，静止不会凭空增加分辨率。
                delta = 0 if self.last_true_theta is None else theta - self.last_true_theta
                phases = phase - self.phase_direction * delta * np.arange(15, -1, -1) / 16
                sample_q4 = int(np.sum(np.rint(self._sensor_code(phases))))
                theta_q10 = ((sample_q4 - self.code_up * 16) * self.slope_q20 * self.phase_direction) >> 10
                if theta_q10 > 3217:
                    theta_q10 -= 6434
                elif theta_q10 < -3217:
                    theta_q10 += 6434
                theta = theta_q10 / 1024
            else:
                self.sensor_fault = True  # 实物电气盲区可产生不可恢复位置跳变，按保护停机。
                theta = self.last_theta if not self.first else wrap(theta)
            self.last_true_theta = x[0]
            arm = (arm_count * (6588397 // self.p.encoder_cpr) >> 10) / 1024
        if self.first:
            self.first = False
        else:
            rate = wrap(theta - self.last_theta) / self.p.dt
            speed = (arm - self.last_arm) / self.p.dt
            if self.quantized:
                theta_step = round(theta * 1024) - round(self.last_theta * 1024)
                if theta_step > 3217:
                    theta_step -= 6434
                elif theta_step < -3217:
                    theta_step += 6434
                rate_q10 = theta_step * 1000
                speed_q10 = ((arm_count - self.last_arm_count) * (6588397 // self.p.encoder_cpr) * 1000) >> 10
                self.q_omega += (rate_q10 - self.q_omega) >> 3
                self.q_speed += (speed_q10 - self.q_speed) >> 3
                self.omega, self.speed = self.q_omega / 1024, self.q_speed / 1024
            else:
                self.omega += (rate - self.omega) / 8
                self.speed += (speed - self.speed) / 8
        self.last_theta, self.last_arm = theta, arm
        self.last_arm_count = arm_count
        return np.array([theta, self.omega, arm, self.speed])

    def update(self, x, elapsed):
        state = self.measured(x)
        if self.sensor_fault or self.protection_fault:
            return 0.0
        theta, omega, arm, speed = state
        if abs(arm) > 6 or abs(speed) > 20 or abs(omega) > 30 or (not self.balance and elapsed >= 15):
            self.protection_fault = True
            return 0.0
        if not self.balance and abs(theta) < math.radians(15) and abs(omega) < 3.5:
            self.balance = True
            self.catch_arm = arm
            self.age = 0
            self.captures += 1
        elif self.balance and abs(theta) > math.radians(40):
            self.balance = False
            self.protection_fault = True
            self.falls += 1
            return 0.0
        if self.balance:
            state[2] -= self.catch_arm  # 基础功能维持捕获时转臂位置，不强拉回开机位置。
            self.age += 1
            if self.quantized:
                q_state = np.rint(state * 1024).astype(np.int64)
                return feedback_q10(q_state, self.gain_q10, self.age) * self.p.supply / 1000
            return float(np.clip(-self.gain @ state, -self.p.supply, self.p.supply))
        if self.quantized:
            return swing_q10(np.rint(state * 1024), kick=elapsed < 0.04) * self.p.supply / 1000
        energy_deficit = 1 - math.cos(theta) - omega**2 / (2 * self.p.gravity / self.p.jp)
        direction = 1 if omega * math.cos(theta) >= 0 else -1
        voltage = 8 * direction * (1 if energy_deficit > 0 else -1) - 0.3 * speed - 0.504 * arm
        if elapsed < 0.04:
            voltage = 8
        return float(np.clip(voltage, -9.6, 9.6))


def simulate(duration=12.0, quantized=True, initial=None, disturbance=None, parameters=None):
    p = parameters or Parameters()
    x = np.array(initial if initial is not None else [math.pi, 0, 0, 0], dtype=float)
    controller = Controller(p, quantized, initial_balance=abs(wrap(x[0])) < 0.1)
    samples = []
    for index in range(round(duration / p.dt)):
        t = index * p.dt
        voltage = controller.update(x, t)
        push = disturbance(t) if disturbance else 0.0
        x = integrate(x, voltage, p, push)
        samples.append([t, wrap(x[0]), x[1], x[2], x[3], voltage, int(controller.balance)])
    return np.array(samples), controller


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=12)
    args = parser.parse_args()
    gain, qgain, poles = design_gain()
    trace, controller = simulate(args.seconds)
    indices = np.flatnonzero(trace[:, 6])
    print(json.dumps({"evidence": "algorithm_simulation_only_not_hardware_acceptance",
                      "state_units": "[rad,rad/s,rad,rad/s] Q10",
                      "sensor_installation_assumption": {"down_electrical_degrees": Parameters().sensor_phase_down_degrees,
                                                          "THETA_SIGN": Parameters().theta_sign,
                                                          "ADC_span_codes": Parameters().adc_span_codes},
                      "gain_volts": gain.tolist(), "gain_permille_q10": qgain.tolist(),
                      "max_closed_loop_pole": float(max(abs(poles))),
                      "capture_seconds": float(trace[indices[0], 0]) if indices.size else None,
                      "captures": controller.captures, "falls": controller.falls,
                      "sensor_fault": controller.sensor_fault,
                      "protection_fault": controller.protection_fault,
                      "final_pendulum_degrees": float(np.degrees(trace[-1, 1])),
                      "final_arm_degrees": float(np.degrees(trace[-1, 3]))}, indent=2))


if __name__ == "__main__":
    main()
