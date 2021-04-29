import numpy as np
from scipy.linalg import solve_continuous_are

from derive import A as sym_A
from derive import B as sym_B
from derive import sym_mat_as_ndarray


class BalanceLQR:
    def __init__(self, sys_params, Q, R_33):
        A = sym_mat_as_ndarray(sym_A, sys_params)
        B = sym_mat_as_ndarray(sym_B, sys_params)

        # Only R[3, 3] affects the solution K. We set the other diagonal elements to be
        # anything nonzero to prevent R from being singular.
        R = np.diag([1, 1, R_33, 1])

        P = solve_continuous_are(A, B, Q, R)
        self.K = np.linalg.inv(R) @ B.T @ P
        self.k = self.K[2]

    def control(self, state):
        state = np.copy(state)
        state[1] = np.mod(state[1], 2 * np.pi)
        res = -self.k @ (state - np.array([0, np.pi, 0, 0]))
        return res
