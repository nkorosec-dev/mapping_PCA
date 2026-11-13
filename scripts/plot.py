import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

DATA_DIR = ROOT / "data" / "generated"
FIGURE_DIR = ROOT / "results" / "figures"

scene = np.loadtxt(DATA_DIR / "scene.csv", delimiter=",", skiprows=1)
plane = np.loadtxt(DATA_DIR / "ransac_inliers.csv", delimiter=",", skiprows=1)
objects = np.loadtxt(DATA_DIR / "ransac_outliers.csv", delimiter=",", skiprows=1)

# Downsampling for visualization only
rng = np.random.default_rng(42)

def sample_points(points, maximum=15000):
    if len(points) <= maximum:
        return points

    indices = rng.choice(len(points), maximum, replace=False)
    return points[indices]

scene_plot = sample_points(scene)
plane_plot = sample_points(plane)
objects_plot = sample_points(objects)

# Figure
fig = plt.figure(figsize=(18, 7), constrained_layout=True)

ax1 = fig.add_subplot(131, projection="3d")
ax2 = fig.add_subplot(132, projection="3d")
ax3 = fig.add_subplot(133, projection="3d")

# 1. Original point cloud
ax1.scatter(
    *scene_plot.T,
    s=0.6,
    alpha=0.4,
    color="tab:gray",
    depthshade=False
)

ax1.set_title(f"Original scene | {len(scene):,} points")

# 2. RANSAC segmentation
ax2.scatter(
    *plane_plot.T,
    s=0.7,
    alpha=0.3,
    color="tab:blue",
    depthshade=False,
    label="Plane"
)

ax2.scatter(
    *objects_plot.T,
    s=2,
    alpha=0.8,
    color="tab:orange",
    depthshade=False,
    label="Remaining points"
)

ax2.set_title(
    f"RANSAC Segmentation | {len(plane):,} inliers"
)

ax2.legend(loc="upper left")

# 3. Object close-up
ax3.scatter(
    *objects_plot.T,
    s=3,
    alpha=0.8,
    color="tab:orange",
    depthshade=False
)

ax3.set_title(f"Object close-up | {len(objects):,} points")

# Camera and axis settings
for ax in [ax1, ax2, ax3]:
    ax.view_init(elev=27, azim=-60)
    ax.set_proj_type("ortho")

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")

    ax.grid(alpha=0.2)

# Full scene limits
low = scene.min(axis=0) - 0.1
high = scene.max(axis=0) + 0.1

for ax in [ax1, ax2]:
    ax.set_xlim(low[0], high[0])
    ax.set_ylim(low[1], high[1])
    ax.set_zlim(low[2], high[2])

    ax.set_box_aspect((2.2, 2.2, 0.9), zoom=1.1)

# Zoom around the object
center = np.median(objects, axis=0)

ax3.set_xlim(center[0] - 0.35, center[0] + 0.35)
ax3.set_ylim(center[1] - 0.35, center[1] + 0.35)
ax3.set_zlim(center[2] - 0.30, center[2] + 0.30)

ax3.set_box_aspect((1, 1, 0.85), zoom=1.15)

fig.suptitle(
    "3D Scene Reconstruction | RANSAC + PCA Segmentation",
    fontsize=16
)

FIGURE_DIR.mkdir(parents=True, exist_ok=True)

fig.savefig(
    FIGURE_DIR / "ransac_segmentation.png",
    dpi=200,
    bbox_inches="tight"
)

plt.show()