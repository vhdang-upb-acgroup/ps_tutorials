import numpy as np
import matplotlib.pyplot as plt

def figure_eight_2d(t, scale=1):
    """Creates a 2D figure-eight trajectory (Lemniscate of Gerono)."""
    x = scale * np.sin(t)
    y = scale * np.sin(t) * np.cos(t)
    return np.vstack((x, y))  # shape: (2, N)

def rotate_2d(trajectory, angle):
    """Rotates 2D trajectory by given angle (in radians)."""
    R = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle),  np.cos(angle)]
    ])
    return R @ trajectory

def plot_2d_trajectories(angles_list, t):
    """Plots multiple rotated 2D figure-eight trajectories."""
    plt.figure(figsize=(8, 8))

    for angle in angles_list:
        traj = figure_eight_2d(t)
        rotated_traj = rotate_2d(traj, angle)
        label = f"{np.degrees(angle):.1f}°"
        plt.plot(rotated_traj[0], rotated_traj[1], label=label)

    plt.title("2D Figure-Eight with Rotations")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis("equal")
    plt.grid(True)
    plt.legend(title="Rotation")
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    t = np.linspace(0, 2 * np.pi, 500)

    # Rotation angles in radians
    angles = [
        0,
        # np.pi / 6,    # 30°
        # np.pi / 4,    # 45°
        # np.pi / 3,    # 60°
        # np.pi / 2,    # 90°
        # -np.pi / 4    # -45°
    ]

    plot_2d_trajectories(angles, t)
