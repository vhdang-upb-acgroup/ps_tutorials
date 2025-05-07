import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def figure_eight_2d(t, scale=1):
    """Generates 2D figure-eight using lemniscate of Gerono."""
    x = scale * np.sin(t)
    y = scale * np.sin(t) * np.cos(t)
    return x, y

def figure_eight_3d(t, scale=1, z_scale=0.5):
    """Extends 2D figure-eight into 3D with a wavy z component."""
    x, y = figure_eight_2d(t, scale)
    z = z_scale * np.sin(2 * t)
    return np.vstack((x, y, z))

def rotate_3d(trajectory, angles):
    """Rotates a 3D trajectory using Euler angles (roll, pitch, yaw)."""
    alpha, beta, gamma = angles  # roll (x), pitch (y), yaw (z)

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(alpha), -np.sin(alpha)],
        [0, np.sin(alpha),  np.cos(alpha)]
    ])

    Ry = np.array([
        [np.cos(beta), 0, np.sin(beta)],
        [0, 1, 0],
        [-np.sin(beta), 0, np.cos(beta)]
    ])

    Rz = np.array([
        [np.cos(gamma), -np.sin(gamma), 0],
        [np.sin(gamma),  np.cos(gamma), 0],
        [0, 0, 1]
    ])

    R = Rz @ Ry @ Rx
    return R @ trajectory

def plot_3d_trajectories(clamp_angles_list, t):
    """Plots multiple rotated figure-eight trajectories in 3D."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    for angles in clamp_angles_list:
        traj = figure_eight_3d(t)
        rotated_traj = rotate_3d(traj, angles)
        ax.plot(*rotated_traj, label=f'Angles (deg): {np.round(np.degrees(angles), 1)}')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title("3D Figure-Eight Trajectories with Clamping Angles")
    ax.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    t = np.linspace(0, 2 * np.pi, 500)

    # Define rotation angles in radians: (roll, pitch, yaw)
    angles_list = [
        (0, 0, 0),
        (np.pi/6, 0, 0),               # roll
        (0, np.pi/6, 0),               # pitch
        (0, 0, np.pi/6),               # yaw
        (np.pi/6, np.pi/6, np.pi/6),   # combo
        (-np.pi/4, np.pi/8, np.pi/4)   # tilted the other way
    ]

    plot_3d_trajectories(angles_list, t)
