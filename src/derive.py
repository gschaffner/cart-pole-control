import numpy as np
from sympy import Matrix, Symbol, cos, pi, pprint, sin
from sympy.abc import I, L, M, g, m, omega, theta, u, v, x

import sim

q = Matrix([x, theta])
q_dot = Matrix([v, omega])

H = Matrix([[M + m, m * L * cos(theta)], [m * L * cos(theta), I]])
C = Matrix([[0, -m * L * sin(theta) * omega], [0, 0]])
G = Matrix([0, m * g * L * sin(theta)])

q_ddot = H.inv() * (Matrix([u, 0]) - C * q_dot - G)
q_ddot.simplify()

# linearize for LQR
s = Matrix([q, q_dot])
s_dot = Matrix([q_dot, q_ddot])
fixed_point = {theta: pi, omega: 0, u: 0}

A = s_dot.jacobian(s).subs(fixed_point)
A.simplify()

dummy = Symbol("dummy")
B = s_dot.jacobian(Matrix([dummy, dummy, u, dummy])).subs(fixed_point)
B.simplify()


def sym_mat_as_ndarray(matrix, params):
    return np.array(
        matrix.subs({g: sim.g, M: params.M, m: params.m, L: params.L, I: params.I})
    ).astype(np.float64)


if __name__ == "__main__":
    print("q_ddot=")
    pprint(q_ddot)
    print("A=")
    pprint(A)
    print("B=")
    pprint(B)

    # validate manual differentiation in derivation document
    A_bottomleft = (-H.inv() * G.jacobian(q)).subs(fixed_point)
    assert A_bottomleft.equals(A[2:4, 0:2])
    B_bottom3 = (H.inv()[:, 0]).subs(fixed_point)
    assert B_bottom3.equals(B[2:4, 2])
