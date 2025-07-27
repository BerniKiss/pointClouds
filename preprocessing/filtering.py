import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("preprocessing/point_cloud.ply")
print("Point cloud readed:", pcd)

o3d.visualization.draw_geometries([pcd])

pcd.estimate_normals()
normals = np.asarray(pcd.normals)
colors = np.abs(normals)
pcd.colors = o3d.utility.Vector3dVector(colors)
o3d.visualization.draw_geometries([pcd])


# nb_points: if a point has LESS THAN  minimum nb_points then it is consdiered a NOISE and REMOVES
# radius: the distance in which we want to apply
clean, ind = pcd.remove_radius_outlier(nb_points=20, radius=0.05)

# ind: the removed points
# print(f"Removed points: {ind}")

print(f"Cleand point clouds: {len(clean.points)} points")
o3d.visualization.draw_geometries([clean])


# comparing the cleaned and the real
pcd.paint_uniform_color([0.9, 0.1, 0.1])  # real one red
clean.paint_uniform_color([0.1, 0.9, 0.1])  # Tcleand one green

# because the points are too dense
pcd = pcd.voxel_down_sample(voxel_size=0.05)
# clean = clean.voxel_down_sample(voxel_size=0.05)

o3d.visualization.draw_geometries([pcd, clean])
