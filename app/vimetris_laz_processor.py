import laspy
import open3d as o3d
import numpy as np

# Leggi il file .laz
las_file = laspy.read("example.laz")

# Estrai le coordinate dei punti
points = np.vstack((las_file.x, las_file.y, las_file.z)).transpose()

# Crea un oggetto PointCloud di Open3D
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(points)

# Visualizza la nuvola di punti
o3d.visualization.draw_geometries([point_cloud])