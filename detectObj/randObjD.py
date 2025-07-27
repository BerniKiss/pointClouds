import numpy as np
import open3d as o3d
from scipy.optimize import least_squares
from genPCD import generate__point_cloud

# RANSAC SPHERE DETECTIOn
def fit_sphere(points):
    def residuals(params, points):
        x0, y0, z0, r = params
        return np.sqrt((points[:, 0] - x0)**2 + (points[:, 1] - y0)**2 + (points[:, 2] - z0)**2) - r

    x0, y0, z0 = np.mean(points, axis=0)
    r0 = np.mean(np.linalg.norm(points - [x0, y0, z0], axis=1))
    initial_guess = [x0, y0, z0, r0]

    result = least_squares(residuals, initial_guess, args=(points,))
    return result.x

def ransac_sphere(points, threshold=0.01, iterations=1000):
    best_inliers = []
    best_params = None

    for _ in range(iterations):
        # 4 random points
        sample = points[np.random.choice(points.shape[0], 4, replace=False)]
        params = fit_sphere(sample)

        inliers = []
        for point in points:
            # calcualte distance
            distance = np.abs(np.sqrt((point[0] - params[0])**2 +
                                    (point[1] - params[1])**2 +
                                    (point[2] - params[2])**2) - params[3])
            if distance < threshold:
                inliers.append(point)

        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_params = params

    return best_params, np.array(best_inliers)


# ransacx plain
def fit_plane(points):
    centroid = np.mean(points, axis=0)
    _, _, vh = np.linalg.svd(points - centroid)
    normal = vh[2, :]
    d = -np.dot(normal, centroid)
    return np.append(normal, d)

def ransac_plane(points, threshold=0.01, iterations=500):
    best_inliers = []
    best_params = None

    for _ in range(iterations):
        # 3 random points
        sample = points[np.random.choice(points.shape[0], 3, replace=False)]
        params = fit_plane(sample)

        inliers = []
        for point in points:
            # calculate distance
            distance = np.abs(np.dot(params[:3], point) + params[3]) / np.linalg.norm(params[:3])
            if distance < threshold:
                inliers.append(point)

        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_params = params

    return best_params, np.array(best_inliers)


# VISUALIYATION
def visualize_detection_results(original_pcd, plane_inliers, sphere_inliers, remaining_points):
    # colors
    plane_pcd = o3d.geometry.PointCloud()
    plane_pcd.points = o3d.utility.Vector3dVector(plane_inliers)
    plane_pcd.paint_uniform_color([1, 0, 0])  # red -plane

    sphere_pcd = o3d.geometry.PointCloud()
    sphere_pcd.points = o3d.utility.Vector3dVector(sphere_inliers)
    sphere_pcd.paint_uniform_color([0, 1, 0])  # green - sphere

    remaining_pcd = o3d.geometry.PointCloud()
    remaining_pcd.points = o3d.utility.Vector3dVector(remaining_points)
    remaining_pcd.paint_uniform_color([0, 0, 1])  # blue - remained


    o3d.visualization.draw_geometries([plane_pcd, sphere_pcd, remaining_pcd],
                                      window_name="RANSAC",
                                      width=800, height=600)


def main():
    print("generating point clouds..")
    pcd = generate__point_cloud(3000)
    points = np.asarray(pcd.points)
    print(f"   generated points number: {len(points)}")

    o3d.visualization.draw_geometries([pcd], window_name="Original")

    print(" RANSAC plane detect...")
    plane_params, plane_inliers = ransac_plane(points, threshold=0.05, iterations=2000)
    print(f"   Plain parameters (a, b, c, d): {plane_params}")
    print(f"   Plain points number: {len(plane_inliers)}")

    # remaining points
    plane_inlier_indices = []
    for i, point in enumerate(points):
        distance = np.abs(np.dot(plane_params[:3], point) + plane_params[3]) / np.linalg.norm(plane_params[:3])
        if distance < 0.05:
            plane_inlier_indices.append(i)

    remaining_points = np.delete(points, plane_inlier_indices, axis=0)
    print(f"Remaining points after detecting: {len(remaining_points)}")

    # sphere
    print("Sphere detection...")
    if len(remaining_points) > 4:
        sphere_params, sphere_inliers = ransac_sphere(remaining_points, threshold=0.05, iterations=2000)
        print(f"   sphere parameters (x0, y0, z0, r): {sphere_params}")
        print(f"   sphere points number: {len(sphere_inliers)}")

        # remining points
        sphere_inlier_indices = []
        for i, point in enumerate(remaining_points):
            distance = np.abs(np.sqrt((point[0] - sphere_params[0])**2 +
                                    (point[1] - sphere_params[1])**2 +
                                    (point[2] - sphere_params[2])**2) - sphere_params[3])
            if distance < 0.05:
                sphere_inlier_indices.append(i)

        final_remaining = np.delete(remaining_points, sphere_inlier_indices, axis=0)
        print(f"Vremaining points: {len(final_remaining)}")
        visualize_detection_results(pcd, plane_inliers, sphere_inliers, final_remaining)

    else:
        print(" Not enough points for detectins sphere!")
        sphere_inliers = np.array([])
        final_remaining = remaining_points
        visualize_detection_results(pcd, plane_inliers, sphere_inliers, final_remaining)


if __name__ == "__main__":
    main()