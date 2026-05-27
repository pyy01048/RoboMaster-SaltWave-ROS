import numpy as np
from cbf_safety_filter import CBFSafetyFilter

def test_cbf_safety_filter():
    obstacles = [
        np.array([5.0, 0.0, 1.0]),
        np.array([-5.0, 5.0, 1.0]),
        np.array([0.0, -5.0, 1.0]),
    ]
    safety_filter = CBFSafetyFilter(obstacles, alpha=1.0)

    x_start = np.array([0.0, 0.0])
    x_goal = np.array([10.0, 0.0])

    dt = 0.1
    T = 200

    x_traj = np.zeros((T, 2))
    u_traj = np.zeros((T, 2))
    h_traj = np.zeros(T)

    x = x_start.copy()
    for t in range(T):
        u_nominal = 0.3 * (x_goal - x) / np.linalg.norm(x_goal - x)
        u_safe = safety_filter.compute_safe_control(x, u_nominal)
        x = x + u_safe * dt

        x_traj[t] = x
        u_traj[t] = u_safe
        h_traj[t] = safety_filter.h(x)

    print("x_start:", x_start)
    print("x_goal:", x_goal)
    print("x_final:", x_traj[-1])
    print("h_traj[-10:]:", h_traj[-10:])
    print("h_traj[-1]:", h_traj[-1])
    assert np.allclose(x_traj[-1], x_goal, atol=0.5), "The final state is not close to the goal state."

if __name__ == "__main__":
    test_cbf_safety_filter()
