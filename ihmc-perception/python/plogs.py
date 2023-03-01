import h5py
import cv2
import numpy as np
import os
from hdf5_reader import *
from hdf5_converter import *
import argparse
from transform_utils import *
import math

def convert_main():

    # Simulation:
    # 20230225_235428_PerceptionLog.hdf5

    # Real:
    # 20221212_184748_PerceptionLog.hdf5
    # 20221212_184906_PerceptionLog.hdf5
    # 20221212_184940_PerceptionLog.hdf5

    # 20221216_141954_PerceptionLog.hdf5
    # 20221216_143619_PerceptionLog.hdf5
    # 20221216_144027_PerceptionLog.hdf5

    home = os.path.expanduser('~')

    # path = home + '/.ihmc/logs/perception/'

    path = '/home/bmishra/Workspace/Data/LogData/Perception/'

    old_file = '20230207_214209_PerceptionLog.hdf5'
    new_file = old_file.replace('.hdf5', 'Fixed.hdf5')


    data = h5py.File(path + old_file, 'r')

    channels = collect_channels(data)

    conversions = []

    for channel in channels:
        conversions.append(dict(dtype=channel['dtype'], src_name=channel['name'], dst_name=channel['name'], count=channel['count']))


    # conversions = [dict(type='byte', src_name='ouster/depth/', dst_name='ouster/depth/', count=507),
    #                dict(type='float', src_name='ouster/sensor/position/', dst_name='ouster/sensor/position/', count=50),
    #                dict(type='float', src_name='ouster/sensor/orientation/', dst_name='ouster/sensor/orientation/', count=50)]

    convert_hdf5_file(path, old_file, new_file, conversions)



def player_main(data, indices = None):

    print(data.keys())

    if indices is None:

        for i in range(len(data['l515/depth/'])):

            # print("Showing image: ", i)
            display_image(data, i, 'l515/depth/', 20)

    else:
            
            for i in indices:
    
                # print("Showing image: ", i)
                display_image(data, i, 'l515/depth/', 20)
    

def filter_by_motion(mocap_position, mocap_orientation, sensor_position, sensor_orientation):

    # Extract frame indices in which there is significant motion, assuming there may be multiple such sections
    indices = []
    for i in range(1, mocap_position.shape[0]):
        if np.linalg.norm(mocap_position[i] - mocap_position[i-1]) > 0.002:
            indices.append(i)

    # Extract mocap and sensor position and orientation for each index
    mocap_position = mocap_position[indices]
    mocap_orientation = mocap_orientation[indices]
    sensor_position = sensor_position[indices]
    sensor_orientation = sensor_orientation[indices]

    return (mocap_position, mocap_orientation, sensor_position, sensor_orientation)

def plotter_main(data):

    mocap_position = get_data(data, 'mocap/rigid_body/position/')
    mocap_orientation = get_data(data, 'mocap/rigid_body/orientation/')
    sensor_position = get_data(data, 'l515/sensor/position/')
    sensor_orientation = get_data(data, 'l515/sensor/orientation/')

    mocap_euler = convert_quaternions_to_euler_angles(mocap_orientation)
    sensor_euler = convert_quaternions_to_euler_angles(sensor_orientation)


    # Subtract PI if greater than 0 and add PI if less than 0 to both euler angle arrays
    mocap_euler[:,0] = -mocap_euler[:,0]
    for i in range(mocap_euler.shape[0]):
            if mocap_euler[i, 0] > 0:
                mocap_euler[i, 0] -= np.pi / 2
            else:
                mocap_euler[i, 0] += np.pi / 2

    mocap_euler[:,2] = -mocap_euler[:,2]
    for i in range(mocap_euler.shape[0]):
            if mocap_euler[i, 2] > 0:
                mocap_euler[i, 2] -= np.pi * 2

    # Filter out spikes in Z-euler
    for i in range(1, mocap_euler.shape[0]):
        if math.fabs(mocap_euler[i, 2] - mocap_euler[i-1, 2]) > 0.5:
            mocap_euler[i, 2] = mocap_euler[i - 1, 2]
        

    # Remove offset in Y angles, align mocap to sensor, after reversing mocap phase
    mocap_euler[:, 1] = -mocap_euler[:, 1]
    mocap_euler[:, 1] -= mocap_euler[0, 1] - sensor_euler[0, 1]

    
    transform = compute_icp_transform(mocap_position[:200, :3], sensor_position[:200, :3])
    # transform = get_relative_transform_se3(mocap_position[0], mocap_orientation[0], sensor_position[0], sensor_orientation[0])

    print("Shapes Mocap Position: ", mocap_position.shape, " Sensor Position: ", sensor_position.shape, " Mocap Orientation: ", mocap_orientation.shape, " Sensor Orientation: ", sensor_orientation.shape)

    print("Transform: ", transform)

    #  Transform mocap position and orientation to bring them to sensor frame
    #  transform is in SE(3) and mocap_position is in R^3, and mocap_orientation is in R^4 with ordering (X, Y, Z, W)
    positions = mocap_position.T
    ones = np.ones(shape=(1, positions.shape[1]))
    positions = np.vstack((positions, ones))
    positions = np.matmul(transform, positions)
    mocap_position = positions[:3, :].T
    mocap_position -= mocap_position[0] - sensor_position[0]
    
    plot_position([sensor_position, mocap_position], ['-r', '-b'], "Estimated State [RED] - Ground Truth [BLUE]", "Position")
    plot_position([sensor_euler, mocap_euler], ['-r', '-b'], "Estimated State [RED] - Ground Truth [BLUE]", "Euler")


if __name__ == '__main__':

    home = os.path.expanduser('~')
    path = home + '/.ihmc/logs/perception/IROS_2023/'
    files = sorted(os.listdir(path))
    
    titles = [  'WalkForward_FallAtTheEnd', # 20230228_191411_PerceptionLog.hdf5
                'Circular_Inward',          # 20230228_195802_PerceptionLog.hdf5
                'Circular_Turn',            # 20230228_195937_PerceptionLog.hdf5
                'Circular_Outward_Fall',    # 20230228_200243_PerceptionLog.hdf5
                'Circular_Outward',         # 20230228_201455_PerceptionLog.hdf5
                'WalkForward_Rough',        # 20230228_201947_PerceptionLog.hdf5
                'Turn_Rough',               # 20230228_202104_PerceptionLog.hdf5
                'WalkBack_Rough',           # 20230228_202456_PerceptionLog.hdf5
                'Stairs_ClimbUp',           # 20230228_204753_PerceptionLog.hdf5
              ]

    for i, file in enumerate(files):
        print('Index: ', i, ' File: ', file, '\tTitle: ', titles[i])

    # INDEX TO LOAD ----------------------------------------------------->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

    index_to_load = 4

    # INDEX TO LOAD -----------------------------------------------------<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    filename = files[index_to_load]
    print('\nLoading file: ', index_to_load, '\tName: ', filename, '\tTitle: ', titles[index_to_load], '\n')

    data = h5py.File(path + filename, 'r')
    

    plotter_main(data)

    player_main(data)
