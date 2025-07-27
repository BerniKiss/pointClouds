import open3d as o3d
import wget
import os

# point cloud file
url = 'https://raw.githubusercontent.com/PointCloudLibrary/pcl/master/test/bunny.pcd'
# filename = wget.download(url)

pcd = o3d.io.read_point_cloud("bunny.pcd")
print(f"Point numbers pcd: {pcd}")
o3d.visualization.draw_geometries([pcd])

# just a cube
pcd2 = o3d.io.read_point_cloud("cube.xyz", format="xyz")
print(f"Ponint numbers pcd2: {pcd2}")
print("Bounding box pcd2:", pcd2.get_axis_aligned_bounding_box())

bbox = pcd2.get_axis_aligned_bounding_box()
bbox.color = (1, 0, 0)
o3d.visualization.draw_geometries([pcd2, bbox])

# bunny mesh
current_dir = os.getcwd()
bunny_path = os.path.join(current_dir, "bunny", "bunny", "reconstruction", "bun_zipper.ply")
mesh = o3d.io.read_triangle_mesh(bunny_path)

# normals
mesh.compute_vertex_normals()
for i, normal in enumerate(mesh.vertex_normals):
    print(f"Normal {i}: {normal}")
# bunny mesh visualized
o3d.visualization.draw_geometries([mesh])