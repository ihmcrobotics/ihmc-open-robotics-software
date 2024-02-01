import h5py
import numpy as np
from hdf5_reader import *
    

def get_data(data, namespace):
    ds = []

    print(data[namespace].keys())

    keys = list(map(str, sorted(list(map(int, data[namespace].keys())))))

    for key in keys:
        array = data[namespace + key][:]
        ds.append(array)

    data_block = np.vstack(ds)
    return data_block

if __name__ == '__main__':
    path = '/home/bmishra/Workspace/Data/Sensor_Logs/Depth/Good/'

    file = '20221216_143619_PerceptionLog.hdf5'

    data = h5py.File(path + file, 'r')

    position = get_data(data, '/l515/sensor/position/')
    
    print(position)

    # position = get_data(data, '/robot/root/position/')
    # orientation = get_data(data, '/robot/root/orientation/')
    
    # joint_angles = get_data(data, '/robot/joint_angles/')
    # joint_velocities = get_data(data, '/robot/joint_velocities/')
    # joint_torques = get_data(data, '/robot/joint_torques/')
    
    plot_position(position, 'Pelvis')
    # plot_joint_angles(orientation)
    # plot_joint_angles(joint_angles)
    # plot_joint_angles(joint_angles)
    # plot_joint_angles(joint_angles)
    # plot_joint_angles(joint_angles)