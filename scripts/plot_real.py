import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

points = np.loadtxt(
    ROOT / "data/generated/real_cloud.csv",
    delimiter=",",
    skiprows=1
)

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

ax.scatter(
    points[::5, 0],
    points[::5, 1],
    points[::5, 2],
    s=0.3
)

ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")

ax.set_title("RealSense D455 | 3D Point Cloud")

ax.set_box_aspect(np.ptp(points, axis=0))
ax.view_init(elev=-65, azim=-90)

plt.show()