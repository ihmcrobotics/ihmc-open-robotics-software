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