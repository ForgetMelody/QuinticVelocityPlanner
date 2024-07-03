#!/usr/bin/python3
"""
Copyright © 2021 boldyoungster. All rights reserved.

@file Polynomial.py
@date: 11:06:12, February 28, 2021
"""

import numpy as np
import matplotlib.pyplot as plt


class PolynomialQuintic:
    def __init__(self, t0, t1, q0, q1, v0=0.0, v1=0.0, a0=0.0, a1=0.0):
        coeffs = self.__ComputeQuinticCoeffs(t0, t1, q0, q1, v0, v1, a0, a1)
        self.poly = np.poly1d(coeffs[::-1])

    @classmethod
    def __ComputeQuinticCoeffs(cls, t0, t1, q0, q1, v0, v1, a0, a1):
        T = t1 - t0
        T2 = T * T
        h = q1 - q0
        k0 = q0
        k1 = v0
        k2 = 0.5 * a0
        k3 = (20. * h - (8. * v1 + 12. * v0) * T -
              (3 * a0 - a1) * T2) / (2. * T * T2)
        k4 = (-30. * h + (14*v1 + 16*v0)*T+(3*a0 - 2*a1)*T2) / (2. * T2 * T2)
        k5 = (12 * h - 6*(v1 + v0) * T + (a1 - a0) * T2) / (2 * T2 * T2 * T)
        return (k0, k1, k2, k3, k4, k5)


if __name__ == "__main__":
    t0, t1 = 0, 2.5
    q0, q1 = 0, 3
    v0, v1 = 0, 0
    a0, a1 = 0, 0
    poly = PolynomialQuintic(t0, t1, q0, q1,
                             v0, v1, a0, a1)
    ts = np.linspace(t0, t1, 200)
    qs = poly.poly(ts)
    vs = poly.poly.deriv(1)(ts)
    dvs = poly.poly.deriv(2)(ts)
    plt.subplots_adjust(hspace=1)
    plt.suptitle("QuinticPolynomial")
    plt.subplot(311)
    plt.plot(ts, qs)
    plt.title("Position")
    plt.subplot(312)
    plt.plot(ts, vs)
    plt.title("Velocity")
    plt.subplot(313)
    plt.plot(ts, dvs)
    plt.title("Acceleration")
    plt.savefig("./QuinticPolynomial.png")
    plt.show()

