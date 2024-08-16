import open3d as o3d
import copy
import torch

from hdf5_reader import *

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    # source_temp.paint_uniform_color([1, 0.506, 0])
    # target_temp.paint_uniform_color([0.3, 0.451, 0.829])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])

def draw_registration_result_tensor(source, target, transformation):
    source_temp = source.clone()
    target_temp = target.clone()

    source_temp.transform(transformation)

    # This is patched version for tutorial rendering.
    # Use `draw` function for you application.
    o3d.visualization.draw_geometries(
        [source_temp.to_legacy(),
         target_temp.to_legacy()],
        zoom=0.4459,
        front=[0.9288, -0.2951, -0.2242],
        lookat=[1.6784, 2.0612, 1.4451],
        up=[-0.3402, -0.9189, -0.1996])

def filter_by_distance(points, near, far):
    # Remove all point sthat are too far or too close
    lateral_dists = np.sqrt(points[:, 0] * points[:, 0] + points[:, 1] * points[:, 1])

    print("Lateral Shape: ", lateral_dists.shape, points.shape)

    condition_within_cylinder = lateral_dists > 0.4
    condition_too_close = points[:, 2] > near
    points = points[condition_too_close]

    condition_too_far = points[:, 2] < far
    points = points[condition_too_far]
    return points

def fast_convert_depth_to_cloud(depth_map, K):
    # Create a grid of pixel coordinates
    h, w = depth_map.shape
    y, x = torch.meshgrid(torch.arange(h), torch.arange(w))

    # Flatten depth and pixel coordinates
    depth_values = depth_map.reshape(-1)  # Flatten the depth map


    X, Y, Z = x.reshape(-1), y.reshape(-1), torch.ones_like(depth_values, dtype=torch.float32)

    print("Min: ", torch.min(depth_values), "Max: ", torch.max(depth_values), "Avg: ", torch.mean(depth_values))

    print(X, Y, depth_values)

    pixel_coordinates = torch.stack([X, Y, Z], dim=0)

    # Calculate the inverse of K
    K_inv = torch.inverse(K)

    print("K_inv: ", K_inv)

    # Multiply the inverse of K with the pixel coordinates
    homo_world_coordinates = torch.matmul(K_inv, pixel_coordinates)

    print(homo_world_coordinates.numpy()[:,:100])

    world_coordinates = homo_world_coordinates * depth_values

    print(world_coordinates.numpy()[:,:100])

    # Transpose to get a list of 3D points (shape: Nx3)
    points = world_coordinates.transpose(0, 1).numpy()

    return points

def get_point_cloud(data, i):
    depth = load_depth(data, i, "/l515/depth/")
    depth_metric = np.array(depth * 0.001, dtype=np.float32)
    depth_map = torch.tensor(depth_metric, dtype=torch.float32)
    points = fast_convert_depth_to_cloud(depth_map, K)
    points = filter_by_distance(points, 0.7, 1.5)
    return points, depth

def convert_depth_to_cloud(depth, params):
    fx, fy, cx, cy = params
    points = []
    for y in range(depth.shape[0]):
        for x in range(depth.shape[1]):
            
            z = depth[y,x] * 0.001
            point = np.array([(x - cx) / fx * z, (y - cy) / fy * z, z])
            points.append(point)
            
    return np.array(points)

def perform_multi_scale_icp(source, target):
    import time

    voxel_sizes = o3d.utility.DoubleVector([0.4, 0.1, 0.05])

    treg = o3d.t.pipelines.registration

    # List of Convergence-Criteria for Multi-Scale ICP:
    criteria_list = [
        treg.ICPConvergenceCriteria(relative_fitness=0.00001, relative_rmse=0.00001, max_iteration=150),
        treg.ICPConvergenceCriteria(0.000001, 0.000001, 85),
        treg.ICPConvergenceCriteria(0.0000001, 0.0000001, 40)
    ]

    # `max_correspondence_distances` for Multi-Scale ICP (o3d.utility.DoubleVector):
    max_correspondence_distances = o3d.utility.DoubleVector([0.4, 0.2, 0.08])

    # Initial alignment or source to target transform.
    init_source_to_target = o3d.core.Tensor.eye(4, o3d.core.Dtype.Float32)

    # Select the `Estimation Method`, and `Robust Kernel` (for outlier-rejection).
    estimation = treg.TransformationEstimationPointToPlane()

    # Save iteration wise `fitness`, `inlier_rmse`, etc. to analyse and tune result.
    callback_after_iteration = lambda loss_log_map : print("Iteration Index: {}, Scale Index: {}, Scale Iteration Index: {}, Fitness: {}, Inlier RMSE: {},".format(
        loss_log_map["iteration_index"].item(),
        loss_log_map["scale_index"].item(),
        loss_log_map["scale_iteration_index"].item(),
        loss_log_map["fitness"].item(),
        loss_log_map["inlier_rmse"].item()))
    
    # Setting Verbosity to Debug, helps in fine-tuning the performance.
    # o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

    s = time.time()

    registration_ms_icp = treg.multi_scale_icp(source, target, voxel_sizes,
                                            criteria_list,
                                            max_correspondence_distances,
                                            init_source_to_target, estimation,
                                            callback_after_iteration)

    ms_icp_time = time.time() - s
    print("Time taken by Multi-Scale ICP: ", ms_icp_time)

    return registration_ms_icp

def perform_point_to_plane_icp(pcd_source, pcd_target, threshold, trans_init):
    pcd_target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05   , max_nn=30))

    reg_p2p = o3d.pipelines.registration.registration_icp(
                    pcd_source, pcd_target, threshold, trans_init,
                    o3d.pipelines.registration.TransformationEstimationPointToPlane())

    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    draw_registration_result(pcd_source, pcd_target, reg_p2p.transformation)

if __name__ == "__main__":

    import sys
    np.set_printoptions(threshold=sys.maxsize)

    list_files()
    
    # Set K to be the 3x3 camera projection matrix using fx, fy, cx and cy
    fx, fy, cx, cy = 654.29, 654.29, 651.14, 361.89
    K = torch.Tensor([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    params = (fx, fy, cx, cy)

    data = load_file('IROS_2023/20230228_201947_PerceptionLog.hdf5')

    source = 0
    target = 80
    threshold = 0.75
    trans_init = np.eye(4)

    points_source, depth_source = get_point_cloud(data, source)
    points_target, depth_target = get_point_cloud(data, target)


    device = o3d.core.Device("CPU:0")
    dtype = o3d.core.float32

    target_pcd_copy = o3d.geometry.PointCloud()
    target_pcd_copy.points = o3d.utility.Vector3dVector(points_target)
    target_pcd_copy.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))

    pcd_source = o3d.t.geometry.PointCloud(device)
    pcd_source.point.positions = o3d.core.Tensor(points_source, dtype, device)


    # Set all source points to be redish gradient
    pcd_target = o3d.t.geometry.PointCloud(device)
    pcd_target.point.positions = o3d.core.Tensor(points_target, dtype, device)
    pcd_target.point.normals = o3d.core.Tensor(np.asarray(target_pcd_copy.normals), dtype, device)

    print(target_pcd_copy.normals)

    min_height = 0.3
    max_height = 1.5

    # Set all source points to be bluish gradient based on height z value
    pcd_source.point.colors = o3d.core.Tensor(np.array([[0.0, 0.0, (points_source[i, 2] - min_height) / max_height] for i in range(len(points_source))]), dtype, device)

    # Set all target points to be redish gradient based on height z value
    pcd_target.point.colors = o3d.core.Tensor(np.array([[(points_target[i, 2] - min_height) / max_height, 0, 0] for i in range(len(points_target))]), dtype, device)
    
    # display_image(data, source, "l515/depth", 0, "Depth Source")
    # display_image(data, target, "l515/depth", 0, "Depth Target")

    # o3d.visualization.draw_geometries([pcd_source, pcd_target])

    result = perform_multi_scale_icp(pcd_source, pcd_target)


    draw_registration_result_tensor(pcd_source, pcd_target, result.transformation)
    
    print("Inlier Fitness: ", result.fitness)
    print("Inlier RMSE: ", result.inlier_rmse)


# ICP Point-to-Point
# RegistrationResult with fitness=1.000000e+00, inlier_rmse=6.316607e-02, and correspondence_set size of 613913
# Access transformation to get result.
# Transformation is:
# [[ 0.99999426 -0.00201182 -0.00272694 -0.02253057]
#  [ 0.0019456   0.99970833 -0.02407237  0.07619161]
#  [ 0.00277458  0.02406693  0.9997065  -0.02947899]
#  [ 0.          0.          0.          1.        ]]

# ICP Point-to-Plane
# RegistrationResult with fitness=1.000000e+00, inlier_rmse=6.536303e-02, and correspondence_set size of 613913
# Access transformation to get result.
# Transformation is:
# [[ 0.99841159 -0.05627566 -0.00271216 -0.01355051]
#  [ 0.05629566  0.99838231  0.00797311  0.0310629 ]
#  [ 0.00225908 -0.00811313  0.99996454 -0.0392928 ]
#  [ 0.          0.          0.          1.        ]]

# ICP Point-to-Plane + Multi-Scale
# Time taken by Multi-Scale ICP:  0.0565340518951416
# Inlier Fitness:  0.7257298031228785
# Inlier RMSE:  0.03553386625053208