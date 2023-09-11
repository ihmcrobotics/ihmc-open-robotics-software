import open3d as o3d
import copy
import torch

from hdf5_reader import *

def draw_registration_result(source, target, transformation):
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
    points = filter_by_distance(points, 0.5, 1.5)
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

    points_source, depth_source = get_point_cloud(data, source)
    points_target, depth_target = get_point_cloud(data, target)


    pcd_source = o3d.geometry.PointCloud()
    pcd_source.points = o3d.utility.Vector3dVector(points_source)

    # Set all source points to be bluish gradient
    pcd_source.colors = o3d.utility.Vector3dVector(np.array([[0, 0, i / len(points_source)] for i in range(len(points_source))]))

    # Set all source points to be redish gradient
    pcd_target = o3d.geometry.PointCloud()
    pcd_target.points = o3d.utility.Vector3dVector(points_target)
    pcd_target.colors = o3d.utility.Vector3dVector(np.array([[i / len(points_target), 0, 0] for i in range(len(points_target))]))
    
    # display_image(data, source, "l515/depth", 0, "Depth Source")
    # display_image(data, target, "l515/depth", 0, "Depth Target")

    o3d.visualization.draw_geometries([pcd_source, pcd_target])
    




