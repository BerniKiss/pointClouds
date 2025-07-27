import open3d as o3d
import numpy as np
from cropping_module import select_points_and_crop

pcd = o3d.io.read_point_cloud("Cropping/bedroom.ply")

print(pcd)
print(np.asarray(pcd.points))
# pcd = pcd.voxel_down_sample(voxel_size=0.05)
'''
o3d.visualization.draw_geometries([pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])
'''

# normals
pcd.estimate_normals()
normals = np.asarray(pcd.normals)
colors = np.abs(normals)
pcd.colors = o3d.utility.Vector3dVector(colors)
'''
o3d.visualization.draw_geometries([pcd],
                                      zoom=0.3412,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024],
                                      window_name="coloring based on normals")
'''

select_points_and_crop(pcd, output_file="Cropping/obj.ply")
obj = o3d.io.read_point_cloud("Cropping/obj.ply")
o3d.visualization.draw_geometries([obj],up=[-0.0694, -0.9768, 0.2024], front=[0.4257, -0.2125, -0.8795])
