import open3d as o3d
import numpy as np

"""
    A module that allows the user to select points and crops the point cloud based on those points.
    The result is saved to the output file.

    arguments:
        pcd:  the input point cloud.
        output_file the gfile name to save the cropped points.
    """


def select_points_and_crop(pcd, output_file="cropped.ply"):

    print("shift + seft mouse: select points")
    print("shift + right mouse: delete selected points")

    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()

    # getting the selected points
    picked_points = vis.get_picked_points()

    if len(picked_points) > 0:
        print(f"Selected points number: {len(picked_points)}")

        # the coordinates of the selected points
        points = np.asarray(pcd.points)
        selected_points = points[picked_points]

        # we need for the bounding box the min and max x y z values
        min_bound = np.min(selected_points, axis=0)
        max_bound = np.max(selected_points, axis=0)

        print(f"Bounding box:")
        print(f"min Bound: [{min_bound[0]:f}, {min_bound[1]:f}, {min_bound[2]:f}]")
        print(f"max Bound: [{max_bound[0]:f}, {max_bound[1]:f}, {max_bound[2]:f}]")

        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)

        cropped_pcd = pcd.crop(bbox)
        print(f"Initial points: {len(pcd.points)}")
        print(f"Cropped points: {len(cropped_pcd.points)}")

        # saving to the input file name
        o3d.io.write_point_cloud(output_file, cropped_pcd)
        print(f"Cropped point cloud saved as: {output_file}")
    else:
        print("No points selected!")

