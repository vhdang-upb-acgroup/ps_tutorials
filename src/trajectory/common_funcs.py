import numpy as np

def figure_eight_trajectory_2d(t, scale=1.0):
    x = scale * np.sin(t)
    y = scale * np.sin(t) * np.cos(t)
    return x, y

def compute_twist_commands_2d(x, y, dt):
    vx = np.gradient(x, dt)
    vy = np.gradient(y, dt)

    linear_vel = np.sqrt(vx**2 + vy**2)
    heading = np.arctan2(vy, vx)
    angular_vel = np.gradient(heading, dt)

    return linear_vel, angular_vel
