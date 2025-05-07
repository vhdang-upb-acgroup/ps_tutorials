import numpy as np
import matplotlib.pyplot as plt

# Parameters
a = 1.0  # size of figure eight
T = 20   # total time
dt = 0.01
time = np.arange(0, T, dt)

# Initialize storage
x = a * np.sin(time)
y = a * np.sin(time) * np.cos(time)

dx = a * np.cos(time)
dy = a * (np.cos(2 * time))

ddx = -a * np.sin(time)
ddy = -2 * a * np.sin(2 * time)

v = np.sqrt(dx**2 + dy**2)
omega = (dx * ddy - dy * ddx) / (v**2 + 1e-8)

# Plot trajectory
plt.plot(x, y)
plt.title("Figure-Eight Trajectory")
plt.axis("equal")
plt.show()
