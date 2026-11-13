import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data/generated"

points = np.loadtxt(
    DATA / "real_height.csv",
    delimiter=",",
    skiprows=1
)

# Height in millimeters
height = points[:, 2] * 1000

# Same distance threshold as RANSAC
is_plane = np.abs(points[:, 2]) < 0.008

plane = points[is_plane]
remaining = points[~is_plane]

fig = plt.figure(figsize=(17, 6), constrained_layout=True)

ax1 = fig.add_subplot(131, projection="3d")
ax2 = fig.add_subplot(132, projection="3d")
ax3 = fig.add_subplot(133)

# 1. Plane-aligned 3D cloud
sample = points[::8]

ax1.scatter(
    sample[:, 0],
    sample[:, 1],
    sample[:, 2] * 1000,
    s=0.5,
    color="gray"
)

ax1.set_title("3D Point Cloud | Plane-aligned")

# 2. RANSAC segmentation
ax2.scatter(
    plane[::8, 0],
    plane[::8, 1],
    plane[::8, 2] * 1000,
    s=0.5,
    color="tab:blue",
    label="Plane"
)

ax2.scatter(
    remaining[:, 0],
    remaining[:, 1],
    remaining[:, 2] * 1000,
    s=1,
    color="tab:orange",
    label="Remaining"
)

ax2.set_title("RANSAC + PCA")
ax2.legend()

# 3. Top-down height map
scatter = ax3.scatter(
    points[::3, 0],
    points[::3, 1],
    c=height[::3],
    cmap="viridis",
    vmin=-5,
    vmax=40,
    s=0.5
)

ax3.set_title("Height Map | Top View")
ax3.set_xlabel("X [m]")
ax3.set_ylabel("Y [m]")
ax3.set_aspect("equal")

fig.colorbar(
    scatter,
    ax=ax3,
    label="Height above plane [mm]"
)

# 3D view settings
for ax in [ax1, ax2]:
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Height [mm]")

    ax.set_zlim(-10, 45)

    ax.set_box_aspect((1, 0.8, 0.2))
    ax.view_init(elev=30, azim=-65)

plt.show()