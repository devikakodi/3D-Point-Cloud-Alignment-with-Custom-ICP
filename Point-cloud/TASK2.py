import numpy as np
import open3d as o3d
import copy

# Visualization function 
def draw_registration_result(source, target, transformation):
    """
        param: source - source point cloud
        param: target - target point cloud
        param: transformation - 4 X 4 homogeneous transformation matrix
    """
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
        zoom=0.4459,
        front=[0.9288, -0.2951, -0.2242],
        lookat=[1.6784, 2.0612, 1.4451],
        up=[-0.3402, -0.9189, -0.1996])

# Correspondence search function 
def search_for_correspondences(source_cloud, target_cloud, nearest_neigh_num=1):
    """
    Find the closest neighbor for each target point using KDTree.
    """
    point_cloud_tree = o3d.geometry.KDTreeFlann(source_cloud)
    points_arr = []
    for point in target_cloud.points:
        [_, idx, _] = point_cloud_tree.search_knn_vector_3d(point, nearest_neigh_num)
        points_arr.append(source_cloud.points[idx[0]])
    return np.asarray(points_arr)

# Custom ICP implementation with updated convergence criteria
def ICP_implementation(source, target, max_iterations=50, tolerance=1e-6):
    """
    Custom ICP implementation with nearest neighbor search, SVD, and iterative refinement.
    """
    source = copy.deepcopy(source)  # Create a copy to preserve the original
    target = copy.deepcopy(target)  # Create a copy to preserve the original
    transform_matrix = np.eye(4)    # Start with an identity matrix

    prev_error = float('inf')
    for iteration in range(max_iterations):
        # Step 1: Find nearest neighbors
        target_points = np.asarray(target.points)
        new_source_points = search_for_correspondences(source, target, 1)

        # Step 2: Compute centroids
        source_centroid = np.mean(new_source_points, axis=0)
        target_centroid = np.mean(target_points, axis=0)

        # Center the points
        source_centered = new_source_points - source_centroid
        target_centered = target_points - target_centroid

        # Step 3: Compute rotation and translation using SVD
        H = np.dot(source_centered.T, target_centered)
        U, _, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = np.dot(Vt.T, U.T)
        t = target_centroid - np.dot(R, source_centroid)

        # Step 4: Update transformation matrix
        transform_step = np.eye(4)
        transform_step[:3, :3] = R
        transform_step[:3, 3] = t
        transform_matrix = np.dot(transform_step, transform_matrix)

        # Step 5: Transform the source cloud
        source_points = np.asarray(source.points)
        source_points = (np.dot(R, source_points.T) + t.reshape(-1, 1)).T
        source.points = o3d.utility.Vector3dVector(source_points)

        # Step 6: Compute mean squared error (MSE) for convergence
        mse_error = np.mean(np.linalg.norm(target_points - new_source_points, axis=1))
        print(f"Iteration {iteration + 1}, MSE Error: {mse_error}")

        # Check convergence
        if abs(prev_error - mse_error) < tolerance:
            print("Convergence achieved!")
            break
        prev_error = mse_error

    return transform_matrix

# Main execution
if __name__ == "__main__":
    # Load point clouds
    demo_icp_pcds = o3d.data.DemoICPPointClouds()
    source = o3d.io.read_point_cloud('/Users/devikakodi/Desktop/perception_hw2/Q2/kitti_frame1.pcd')
    target = o3d.io.read_point_cloud('/Users/devikakodi/Desktop/perception_hw2/Q2/kitti_frame2.pcd')

    # Visualize initial alignment
    print("Initial alignment:")
    draw_registration_result(source, target, np.eye(4))

    # Apply custom ICP
    print("Running ICP...")
    final_transformation = ICP_implementation(source, target)

    # Visualize final alignment
    print("Final alignment:")
    draw_registration_result(source, target, final_transformation)