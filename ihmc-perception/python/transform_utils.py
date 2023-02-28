import numpy as np
from scipy.spatial.transform import Rotation as R

def get_relative_transform_se3(source_position, source_quaternion, dest_position, dest_quaternion):

    # Compute the SE3 transform from source to dest
    source_rotation = R.from_quat(source_quaternion)
    dest_rotation = R.from_quat(dest_quaternion)
    source_to_dest_rotation = dest_rotation * source_rotation.inv()
    source_to_dest_translation = dest_position - source_position
    source_to_dest_transform = np.eye(4)
    source_to_dest_transform[:3, :3] = source_to_dest_rotation.as_matrix()
    source_to_dest_transform[:3, 3] = source_to_dest_translation
    return source_to_dest_transform

def compute_icp_transform(src_points, target_points):
    """
    Compute the ICP transform from source to target
    :param src_points: source points
    :param target_points: target points
    :return: the ICP transform
    """
    # Compute the centroid of the source and target points
    src_centroid = np.mean(src_points, axis=0)
    target_centroid = np.mean(target_points, axis=0)

    # Compute the covariance matrix
    src_points = src_points - src_centroid
    target_points = target_points - target_centroid
    covariance = np.matmul(src_points.T, target_points)

    # Compute the SVD of the covariance matrix
    U, S, V = np.linalg.svd(covariance)

    # Compute the rotation matrix
    R = np.matmul(V.T, U.T)

    # Compute the translation vector
    t = target_centroid - np.matmul(R, src_centroid)

    # Compute the ICP transform
    icp_transform = np.eye(4)
    icp_transform[:3, :3] = R
    icp_transform[:3, 3] = t
    return icp_transform

def convert_quaternions_to_euler_angles(quaternions):
    # N X 4 numpy array of quaternions to N X 3 numpy array of euler angles
    
    euler_angles = np.zeros((quaternions.shape[0], 3))
    for i in range(quaternions.shape[0]):
        euler_angles[i] = R.from_quat(quaternions[i]).as_euler('xyz', degrees=False)
    
    return euler_angles
    