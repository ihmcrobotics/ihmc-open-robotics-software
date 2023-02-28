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

    path = home + '/.ihmc/logs/perception/'


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



def play_main(indices = None):

    home = os.path.expanduser('~')

    path = home + '/.ihmc/logs/perception/'


    data = h5py.File(path + '20230227_193535_PerceptionLog.hdf5', 'r')

    print(data.keys())

    if indices is None:

        for i in range(2, 10):

            print("Showing image: ", i)
            display_image(data, i, 'l515/depth/', 20)

    else:
            
            for i in indices:
    
                print("Showing image: ", i)
                display_image(data, i, 'l515/depth/', 20)
    

def plot_main():

    home = os.path.expanduser('~')

    path = home + '/.ihmc/logs/perception/'


    data = h5py.File(path + '20230227_193535_PerceptionLog.hdf5', 'r')


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

    #  Perform inverse transform on mocap position and orientation to bring it to sensor frame
    #  transform is in SE(3) and mocap_position is in R^3, and mocap_orientation is in R^4 with ordering (X, Y, Z, W)

    positions = mocap_position.T

    # Append a row of ones to the end of the position vector
    ones = np.ones(shape=(1, positions.shape[1]))
    positions = np.vstack((positions, ones))

    
    # Transform the position vector
    positions = np.matmul(transform, positions)


    # # Apply another rotation to invert X and Z phases
    # rotation = np.eye(4)
    # rotation[0, 0] = -1
    # rotation[2, 2] = -1
    # positions = np.matmul(rotation, positions)

    # Put first three rows back into mocap_position
    mocap_position = positions[:3, :].T
    
    # Remove offset in first position between sensor_position and mocap_position (fix to sensor_position)
    mocap_position -= mocap_position[0] - sensor_position[0]
    

    # # Extract frame indices in which there is significant motion, assuming there may be multiple such sections
    # indices = []
    # for i in range(1, mocap_position.shape[0]):
    #     if np.linalg.norm(mocap_position[i] - mocap_position[i-1]) > 0.002:
    #         indices.append(i)

    # # Extract mocap and sensor position and orientation for each index
    # mocap_position = mocap_position[indices]
    # mocap_orientation = mocap_orientation[indices]
    # sensor_position = sensor_position[indices]
    # sensor_orientation = sensor_orientation[indices]
    

    plot_position([sensor_position, mocap_position], ['-r', '-b'], "Estimated State [RED] - Ground Truth [BLUE]", "Position")
    plot_position([sensor_euler, mocap_euler], ['-r', '-b'], "Estimated State [RED] - Ground Truth [BLUE]", "Euler")

    # play_main(indices)



if __name__ == '__main__':
    plot_main()
