import numpy as np

class CBFSafetyFilter:
    def __init__(self, obstacles, alpha=1.0):
        self.obstacles = obstacles
        self.alpha = alpha

    def h(self, x):
        min_dist = float('inf')
        for obs in self.obstacles:
            dist = np.linalg.norm(x[:2] - obs[:2]) - obs[2]
            if dist < min_dist:
                min_dist = dist
        return min_dist

    def dh_dx(self, x):
        dh_dx = np.zeros(2)
        min_dist = float('inf')
        closest_obs_idx = -1

        for i, obs in enumerate(self.obstacles):
            dist = np.linalg.norm(x[:2] - obs[:2]) - obs[2]
            if dist < min_dist:
                min_dist = dist
                closest_obs_idx = i

        if closest_obs_idx != -1:
            obs = self.obstacles[closest_obs_idx]
            dh_dx = (x[:2] - obs[:2]) / np.linalg.norm(x[:2] - obs[:2])

        return dh_dx

    def compute_safe_control(self, x, u_des):
        """
        Compute the safe control input using quadratic programming.
        x: current state [x, y]
        u_des: desired control input [vx, vy]
        """
        h_x = self.h(x)
        dh_dx = self.dh_dx(x)
        lhs = np.outer(dh_dx, dh_dx) + 1e-6 * np.eye(2)
        rhs = -self.alpha * h_x * dh_dx - np.dot(dh_dx, u_des) * dh_dx

        try:
            u_cbf = np.linalg.solve(lhs, rhs)
        except np.linalg.LinAlgError:
            u_cbf = np.zeros_like(u_des)

        u_safe = u_des + u_cbf
        u_safe = np.clip(u_safe, -0.5, 0.5)
        return u_safe
