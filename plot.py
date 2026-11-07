import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

# Data
reference = np.loadtxt("reference.csv", delimiter=",", skiprows=1)
measured = np.loadtxt("measured.csv", delimiter=",", skiprows=1)
nx, ny, nz, d, angle_error, rmse, _ = np.loadtxt(
    "plane_fit.csv", delimiter=",", skiprows=1
)

normal = np.array([nx, ny, nz])
normal /= np.linalg.norm(normal)
centroid = measured.mean(axis=0)

# PCA plane: nx*x + ny*y + nz*z + d = 0
x = np.linspace(reference[:, 0].min(), reference[:, 0].max(), 30)
y = np.linspace(reference[:, 1].min(), reference[:, 1].max(), 30)
X, Y = np.meshgrid(x, y)
Z = -(nx * X + ny * Y + d) / nz

# Reference grid
N = int(np.sqrt(len(reference)))
Xr, Yr, Zr = [reference[:, i].reshape(N, N) for i in range(3)]

# Point-to-plane distances
residuals = measured @ normal + d

fig = plt.figure(figsize=(16, 7), constrained_layout=True)

# 3D reconstruction
ax = fig.add_subplot(121, projection="3d")

ax.scatter(
    *measured.T, s=8, alpha=0.5,
    color="tab:orange", depthshade=False
)
ax.plot_surface(X, Y, Z, color="tab:green", alpha=0.3, shade=False)
ax.plot_wireframe(
    Xr, Yr, Zr, rstride=5, cstride=5,
    color="tab:blue", linewidth=1.2
)
ax.scatter(*centroid, color="black", s=70)

# Normal direction is arbitrary; orient upward for display.
display_normal = normal if normal[2] >= 0 else -normal
ax.quiver(
    *centroid, *display_normal,
    length=0.5, normalize=True,
    color="tab:red", linewidth=2.5
)

ax.set(
    title=f"PCA Reconstruction | Angular error: {angle_error:.4f}°",
    xlabel="X [m]",
    ylabel="Y [m]",
    zlabel="Z [m]"
)

bounds = np.vstack((reference, measured))
lo, hi = bounds.min(axis=0) - 0.12, bounds.max(axis=0) + 0.12

ax.set_xlim(lo[0], hi[0])
ax.set_ylim(lo[1], hi[1])
ax.set_zlim(lo[2], hi[2])
ax.set_box_aspect(hi - lo)
ax.view_init(elev=28, azim=-65)

ax.legend(handles=[
    Line2D([0], [0], color="tab:blue", lw=2, label="Reference"),
    Line2D([0], [0], marker="o", color="tab:orange",
           linestyle="", label="Measured"),
    Patch(facecolor="tab:green", alpha=0.4, label="PCA plane"),
    Line2D([0], [0], color="tab:red", lw=2, label="Normal"),
    Line2D([0], [0], marker="o", color="black",
           linestyle="", label="Centroid")
], loc="upper left", fontsize=9)

# Residual distribution
ax2 = fig.add_subplot(122)

ax2.hist(
    residuals * 1000, bins=45,
    color="tab:blue", alpha=0.8, edgecolor="white"
)
ax2.axvline(0, color="black", linestyle="--", label="Fitted plane")

ax2.set(
    title=f"Point-to-Plane Residuals | RMSE: {rmse * 1000:.2f} mm",
    xlabel="Signed distance [mm]",
    ylabel="Number of points"
)
ax2.grid(alpha=0.2)
ax2.legend()

fig.suptitle("3D Plane Reconstruction using PCA", fontsize=15)
plt.show()