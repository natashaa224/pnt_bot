import numpy as np
import matplotlib.pyplot as plt

# synthetic wall points
# synthetic wall points (tighter scatter)
np.random.seed(0)
x = np.linspace(0, 3, 150)              # more points
y = 0.09 * np.random.randn(150)         # smaller noise

# rotate scatter by -45 degrees
theta = -45 * np.pi / 180   # radians
R = np.array([[np.cos(theta), -np.sin(theta)],
              [np.sin(theta),  np.cos(theta)]])


points = np.vstack((x, y)).T

points = points @ R.T
# PCA
centroid = points.mean(axis=0)
centered = points - centroid
cov = centered.T @ centered / len(points)
eigvals, eigvecs = np.linalg.eigh(cov)

# principal direction (wall)
dir_vec = eigvecs[:, 1]
# normal
normal = eigvecs[:, 0]

# line for visualization
line = centroid + np.outer(np.linspace(-2, 2, 100), dir_vec)

plt.figure(figsize=(6,6))
plt.scatter(points[:,0], points[:,1], s=10, label="Points")

# PCA line
plt.plot(line[:,0], line[:,1], label="Fitted wall (PCA)", linewidth=2)

# eigenvectors

# eigenvectors
plt.arrow(*centroid, *(dir_vec*1.5),
          width=0.02, color='green', label="Principal axis - Fitted Wall Length")

plt.arrow(*centroid, *(normal*1.0),
          width=0.02, color='red', label="Minor axis - Wall Normal")

# ellipse (covariance)
# theta = np.linspace(0, 2*np.pi, 100)
# ellipse = np.array([np.sqrt(eigvals[1])*np.cos(theta),
#                     np.sqrt(eigvals[0])*np.sin(theta)])
# ellipse_rot = eigvecs @ ellipse + centroid[:,None]


# ---- covariance ellipse (correct scaling) ----
theta = np.linspace(0, 2*np.pi, 100)

# scale factor (2-sigma ~ 95%)
scale = 2.0

# axis lengths
a = scale * np.sqrt(eigvals[1])   # major axis
b = scale * np.sqrt(eigvals[0])   # minor axis

ellipse = np.array([
    b * np.cos(theta),   # minor axis with eigvecs[:,0]
    a * np.sin(theta)    # major axis with eigvecs[:,1]
])
ellipse_rot = eigvecs @ ellipse + centroid[:, None]


plt.plot(ellipse_rot[0], ellipse_rot[1],
         linestyle='--',
         color='orange',
         label="Covariance ellipse (2σ)")


# plt.plot(ellipse_rot[0], ellipse_rot[1], linestyle='--', label="Covariance ellipse")

plt.axis('equal')
plt.legend()
plt.title("PCA Line Fitting Intuition")
plt.show()