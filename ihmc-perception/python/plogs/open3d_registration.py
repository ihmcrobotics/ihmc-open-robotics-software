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

def fast_convert_depth_to_cloud(depth_map, K):
    # Create a grid of pixel coordinates
    h, w = depth_map.shape
    y, x = torch.meshgrid(torch.arange(h), torch.arange(w))

    # Flatten depth and pixel coordinates
    depth_values = depth_map.view(-1)  # Flatten the depth map


    X, Y, Z = x.reshape(-1), y.reshape(-1), torch.ones_like(depth_values)

    print("Min: ", torch.min(depth_values), "Max: ", torch.max(depth_values))

    print(X, Y, depth_values)

    pixel_coordinates = torch.stack([X, Y, Z], dim=0)

    # Calculate the inverse of K
    K_inv = torch.inverse(K)

    # Multiply the inverse of K with the pixel coordinates
    homo_world_coordinates = torch.matmul(K_inv, pixel_coordinates)

    print(homo_world_coordinates)
    print(depth_values)

    world_coordinates = homo_world_coordinates * depth_values

    print(world_coordinates)

    # Transpose to get a list of 3D points (shape: Nx3)
    points = world_coordinates.transpose(0, 1).numpy()

    return points

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

    list_files()
    
    data = load_file('IROS_2023/20230228_201947_PerceptionLog.hdf5')

    depth = load_depth(data, 1, "/l515/depth/")

    # Assuming depth is an OpenCV-loaded NumPy array
    # Make sure it's converted to a PyTorch tensor
    depth_map = torch.tensor(depth, dtype=torch.int16)

    # Set K to be the 3x3 camera projection matrix using fx, fy, cx and cy
    fx, fy, cx, cy = 654.29, 654.29, 651.14, 361.89
    K = torch.Tensor([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    params = (fx, fy, cx, cy)

    points = fast_convert_depth_to_cloud(depth_map, K)

    # Remove all point sthat are too far or too close
    # points = points[points[:, 2] < 4.0]
    # points = points[points[:, 2] > 0.1]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    o3d.visualization.draw_geometries([pcd])




