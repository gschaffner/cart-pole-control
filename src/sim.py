import numpy as np
from scipy.integrate import solve_ivp

g = 9.81


class CartPoleSim:
    """Cart-pole system simulation.

    Attributes
    ----------
    M : float
        Cart mass. In kg.
    m : float
        Pole mass. In kg.
    L : float
        Distance from the rotation axis to the pole's center of mass. In m.
    I : float
        Pole moment of inertia about the axle. In kg m^2.
    state : np.ndarray
        State of the system: np.array([x, theta, x_dot, theta_dot]). The angle theta is
        measured c.c.w. with zero pointing downward. In m, rad, m/s, and rad/s.
    t : float
        Current time of the system. In s.
    fps : int
        Rate of capturing historical state snapshots. In Hz. Set to 0 to disable
        history.
    state_hist : np.ndarray, optional
        Array of historical snapshots of the system state.
    t_hist : np.ndarray, optional
        Times corresponding to snapshots in state_hist. In s.
    """

    def __init__(self, M, m, L, I, state, t=0, fps=30):
        self.M = M
        self.m = m
        self.L = L
        self.I = I
        self.state = np.array(state)  # accept array_like
        self.t = t
        self.fps = fps
        if self.fps != 0:
            self.state_hist = np.copy(self.state).reshape(1, -1)
            self.t_hist = np.array([t])

    def propagate_state(self, control, delta_t):
        """Propagate the state forward by delta_t with constant control u applied.

        Parameters
        ----------
        control : Callable[[np.ndarray], float]
            Callable that takes the system state and returns the control force applied
            to the cart, in kg m/s^2.
        delta_t : float
            Time interval to integrate forward for. In s.
        """

        def state_dot(t, state):
            u = control(state)
            state_dot = np.zeros_like(state)
            state_dot[0] = state[2]
            state_dot[1] = state[3]
            coef = 1 / (
                (self.M + self.m) * self.I - (self.m * self.L * np.cos(state[1])) ** 2
            )
            state_dot[2] = coef * (
                self.m
                * self.L
                * np.sin(state[1])
                * (self.m * g * self.L * np.cos(state[1]) + self.I * state[3] ** 2)
                + self.I * u
            )
            state_dot[3] = (
                coef
                * -self.m
                * self.L
                * (
                    self.m
                    * self.L
                    * np.sin(state[1])
                    * np.cos(state[1])
                    * state[3] ** 2
                    + (self.M + self.m) * g * np.sin(state[1])
                    + u * np.cos(state[1])
                )
            )
            return state_dot

        # To prevent accumulation of errors dependent on the frame rate, we need a
        # snapshot of the state at each frame plus a snapshot at the the final time.
        t_eval = (
            np.arange(self.t_hist[-1], self.t + delta_t, 1 / self.fps)
            if self.fps != 0
            else np.array([])
        )
        t_eval = np.append(t_eval, self.t + delta_t)
        # use default numerical integration method and tolerances
        sol = solve_ivp(
            state_dot, t_span=[self.t, self.t + delta_t], y0=self.state, t_eval=t_eval
        )
        self.state = sol.y.T[-1]
        self.t = sol.t[-1]
        if self.fps != 0:
            self.state_hist = np.concatenate([self.state_hist, sol.y.T[:-1]])
            self.t_hist = np.append(self.t_hist, sol.t[:-1])

    def save_animation(self, filename):
        """Create and save a movie of the historical state.

        Parameters
        ----------
        filename : str
            Output filename, e.g. "animation.mp4". Passed to
            matplotlib.animation.FuncAnimation.save.
        """
        import matplotlib.animation as animation
        import matplotlib.pyplot as plt

        plt.rcParams["font.family"] = "monospace"

        def rad_2_deg(ang):
            return ang * 360 / (2 * np.pi)

        def state_as_str(state, verbose=True):
            if verbose:
                return (
                    f"[{state[0] * 100:+05.1f} cm, "
                    f"{rad_2_deg(np.mod(state[1], 2 * np.pi)):+06.1f}°, "
                    f"{state[2] * 100:+05.1f} cm/s, "
                    f"{rad_2_deg(state[3]):+06.3f}°/s]"
                )
            else:
                return (
                    f"[{state[0] * 100:.1f} cm, "
                    f"{rad_2_deg(np.mod(state[1], 2 * np.pi)):.1f}°, "
                    f"{state[2] * 100:.1f} cm/s, "
                    f"{rad_2_deg(state[3]):.1f}°/s]"
                )

        # hardcode some values for our experimental dimensions
        fig = plt.figure(figsize=(7, 6))
        ax = fig.add_subplot(
            111, xlim=(-0.5, 0.5), ylim=(-0.5, 0.5), xlabel="x [m]", ylabel="y [m]"
        )
        ax.set_aspect("equal")
        ax.grid()

        pendulum_arm = ax.plot([], [], "o-", lw=2)[0]
        dynamic_lbl = ax.text(0.02, 0.98, "", va="top", transform=ax.transAxes)
        ax.set_title(
            r"$[x, \theta, v, \omega]_0$ = "
            f"{state_as_str(self.state_hist[0], verbose=False)}"
        )

        def update_anim(frame):
            state = self.state_hist[frame]
            t = self.t_hist[frame]
            pendulum_arm.set_data(
                [state[0], state[0] + self.L * np.sin(state[1])],
                [0, -self.L * np.cos(state[1])],
            )
            dynamic_lbl.set_text(
                f"t = {t:.2f} s,\n" rf"$[x, \theta, v, \omega]$ = {state_as_str(state)}"
            )
            return pendulum_arm, dynamic_lbl

        anim = animation.FuncAnimation(
            fig,
            update_anim,
            len(self.t_hist),
            # Interval is rounded to the nearest ms here, but this is not problematic
            # since anim.save will use the exact frame rate in the render. This interval
            # would only affect live playback.
            interval=round(1 / self.fps * 1000),
            blit=True,
        )
        anim.save(filename, writer="ffmpeg", fps=self.fps)
