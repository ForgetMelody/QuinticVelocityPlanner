import numpy as np
import time
class PolynomialQuintic:
    def __init__(self, tmax, q0, q1, v0=0.0, v1=0.0, a0=0.0, a1=0.0, max_acc=None, max_vel=None):
        self.t0 = 0
        self.tmax = tmax
        self.q0 = q0
        self.q1 = q1
        self.v0 = v0
        self.v1 = v1
        self.a0 = a0
        self.a1 = a1
        self.max_acc = max_acc
        self.max_vel = max_vel
        self.poly = 0

        # 优化时间跨度
        self.t1 = self.optimize_time()
        
        # 计算五次多项式的系数
        # coeffs = self.ComputeQuinticCoeffs(self.t0, self.t1, self.q0, self.q1, self.v0, self.v1, self.a0, self.a1)
        # self.poly = np.poly1d(coeffs[::-1])

    def ComputeQuinticCoeffs(cls, t0, t1, q0, q1, v0, v1, a0, a1):
        T = t1 - t0
        T2 = T * T
        h = q1 - q0
        k0 = q0
        k1 = v0
        k2 = 0.5 * a0
        k3 = (20. * h - (8. * v1 + 12. * v0) * T - (3 * a0 - a1) * T2) / (2. * T * T2)
        k4 = (-30. * h + (14 * v1 + 16 * v0) * T + (3 * a0 - 2 * a1) * T2) / (2. * T2 * T2)
        k5 = (12 * h - 6 * (v1 + v0) * T + (a1 - a0) * T2) / (2 * T2 * T2 * T)
        return (k0, k1, k2, k3, k4, k5)

    def compute_acc(self, t):
        return (20 * self.poly[5] * t**3 + 12 * self.poly[4] * t**2 + 6 * self.poly[3] * t + 2 * self.poly[2])

    def compute_vel(self, t):
        return (5 * self.poly[5] * t**4 + 4 * self.poly[4] * t**3 + 3 * self.poly[3] * t**2 + 2 * self.poly[2] * t + self.poly[1])

    def check_constraints(self, T):
        # 构造临时多项式系数以进行检查
        coeffs = self.ComputeQuinticCoeffs(self.t0, self.t0 + T, self.q0, self.q1, self.v0, self.v1, self.a0, self.a1)
        self.poly = np.poly1d(coeffs[::-1])

        # 检查最大加速度和最大速度约束
        t = np.linspace(self.t0, self.t0 + T, 100)
        accs = np.polyval(np.polyder(self.poly, 2), t)
        vels = np.polyval(np.polyder(self.poly, 1), t)

        max_acc_current = max(abs(accs))
        max_vel_current = max(abs(vels))

        if self.max_acc is not None and max_acc_current > self.max_acc:
            return False
        if self.max_vel is not None and max_vel_current > self.max_vel:
            return False
        return True

    def optimize_time(self):
        # 使用二分法优化时间
        start_time = time.time()
        T_min, T_max = 0.1, self.tmax  # 初始时间范围
        tolerance = 1e-3  # 容差
        while T_max - T_min > tolerance:
            T_mid = (T_min + T_max) / 2
            if self.check_constraints(T_mid):
                T_max = T_mid
            else:
                T_min = T_mid
        print("Optimization time: {:.5f}s".format(time.time() - start_time))
        return T_max

# 示例使用
tmax = 0
q0 = 0
q1 = 10
v0 = 0
v1 = 0
a0 = 0
a1 = 0
max_acc = 1
max_vel = 2

poly = PolynomialQuintic(tmax, q0, q1, v0, v1, a0, a1, max_acc, max_vel)

# 生成时间点
t = np.linspace(0, poly.t1, 100)

# 生成轨迹
p = poly.poly(t)
v = [poly.compute_vel(ti) for ti in t]
a = [poly.compute_acc(ti) for ti in t]

# 绘制轨迹
import matplotlib.pyplot as plt

plt.figure(figsize=(10, 8))

plt.subplot(3, 1, 1)
plt.plot(t, p, label='Position')
plt.xlabel('Time [s]')
plt.ylabel('Position [m]')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(t, v, label='Velocity', color='orange')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(t, a, label='Acceleration', color='red')
plt.xlabel('Time [s]')
plt.ylabel('Acceleration [m/s²]')
plt.legend()

plt.tight_layout()
plt.show()
