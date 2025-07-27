import open3d as o3d
import numpy as np
from cropping_module import select_points_and_crop

# visualise point clouds
pcd = o3d.io.read_point_cloud("demo2/fragment.ply")
print(pcd)
print(np.asarray(pcd.points))
'''
o3d.visualization.draw_geometries([pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])

'''
#o3d.visualization.draw_geometries([pcd],up=[-0.0694, -0.9768, 0.2024])

# VOXEL
voxel_pcd_size = 0.07
voxel_pcd = pcd.voxel_down_sample(voxel_pcd_size)
'''
o3d.visualization.draw_geometries([voxel_pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])
'''

# VERTEX NORMAL n-nromal points, + - for zooming in or out
# normal vector of all points
# radius -> 0.1 = 10cm
# max_nn ->how many neighbours
# KDTREE an algprithm for knn nearest neighbour
voxel_pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
'''
o3d.visualization.draw_geometries([pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024],
                                  point_show_normal=True)
'''

# accessing the normal of a spesific point
print("Normal vector of the 0th point")
print(pcd.normals[0])

# accessing the normal of the first N points
print("Normal vectors of the first 10 points")
print(np.asarray(pcd.normals)[:10, :])

# cropping an object
# using the written module where you select points
# select_points_and_crop(pcd, output_file="demo2/cropped3.ply")
chair = o3d.io.read_point_cloud("demo2/cropped3.ply")
'''
o3d.visualization.draw_geometries([chair],
                                  zoom=0.7,
                                  front=[0.5439, -0.2333, -0.8060],
                                  lookat=[2.4615, 2.1331, 1.338],
                                  up=[-0.1781, -0.9708, 0.1608])
'''

# painting
print("Paint chair")
chair.paint_uniform_color([1, 0.706, 0])
'''
o3d.visualization.draw_geometries([chair],
                                  zoom=0.7,
                                  front=[0.5439, -0.2333, -0.8060],
                                  lookat=[2.4615, 2.1331, 1.338],
                                  up=[-0.1781, -0.9708, 0.1608])
'''

# distance
dists = pcd.compute_point_cloud_distance(chair)
dists = np.asarray(dists)
ind = np.where(dists > 0.01)[0]
pcd_without_chair = pcd.select_by_index(ind)
'''
o3d.visualization.draw_geometries([pcd_without_chair],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])
'''