#! /usr/bin/env python3

import h5py
import numpy as np
import os
import math
import cv2
import argparse

from hdf5_reader import *
from hdf5_converter import *
from transform_utils import *
from hdf5_generator import *

def convert_main(path, src_file_name, dst_file_name):

    new_file = dst_file_name

    if dst_file_name is None:
        new_file = src_file_name.replace('.hdf5', 'Fixed.hdf5')


    data = h5py.File(path + src_file_name, 'r')

    channels = collect_channels(data)

    conversions = []

    for channel in channels:
        conversions.append(dict(dtype=channel['dtype'], src_name=channel['name'], dst_name=channel['name'], count=channel['count']))

    convert_hdf5_file(path, src_file_name, new_file, conversions)

# Converts all unfixed files with int32 data types to uint8 and gives the option to delete the old version.
def convert_all(path, files):
    for file in files:
        if("Fixed" not in file and ".hdf5" in file):
            convert_main(path, file, None)

    print_file_sizes(path, files)

    if(input("Files are fixed, would you like to delete the old versions? [y/n] ").lower == 'y' or 'yes'):
        for file in files:
            if("Fixed" not in file):
                os.remove(file)



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
    

def plot_data(data):
    position = get_data(data, 'l515/sensor/position/')

    print(position)

    plot_position(0, -1, [position], ['-b'], "Position", "Position")


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

def plotter_main(data, output_file):

    final_positions, final_rotations = extract_trajectory_from_output(output_file)

    # Concate the 100 rows zero 3-vectors on top of the final positions and rotations
    total_zero_rows = 150
    final_positions = np.concatenate((np.zeros((total_zero_rows, 3)), final_positions), axis=0)
    final_rotations = np.concatenate((np.zeros((total_zero_rows, 3)), final_rotations), axis=0)

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

    
    transform = compute_icp_transform(mocap_position, sensor_position)
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
    

    sum_of_squares = 0
    for i in range(150, mocap_position.shape[0]):
        print('Mocap Position: ', mocap_position[i], ' Sensor Position: ', sensor_position[i], ' Euclidean Distance: ', np.linalg.norm(mocap_position[i] - sensor_position[i]))
        sum_of_squares += np.linalg.norm(mocap_position[i] - sensor_position[i]) ** 2

    mean = sum_of_squares / (mocap_position.shape[0] - 150)
    rmse = math.sqrt(mean)

    print("RMSE: ", rmse)

    plot_position(150, -1, [mocap_position, final_positions], ['-b', '-r', '-y'], "Estimated State [RED] - Ground Truth [BLUE]", "Position")

    # plot_position(150, 640, [mocap_euler, final_rotations, sensor_orientation], ['-b', '-r', '-y'], "Estimated State [RED] - Ground Truth [BLUE]", "Euler")


def extract_trajectory_from_output(file):
    
    # Given a line in the following format, extract the position and rotation as separate three vectors, and stack them into two lists:
    # 230301 15:16:28:643 [INFO] (PerceptionPrintTools.java:74): [499] Translation: ( 1.976,  1.221,  1.209 ), Rotation: ( 0.251,  1.340,  1.394 )

    positions = []
    rotations = []

    with open(file, 'r') as f:
        for line in f:
            if "Translation" in line:
                position = line.split('Translation: (')[1].split(')')[0].split(',')
                position = [float(p) for p in position]
                positions.append(position)

            if "Rotation" in line:
                rotation = line.split('Rotation: (')[1].split(')')[0].split(',')
                rotation = [float(r) for r in rotation]
                rotations.append(rotation)

    # Stack them as rows in matrices
    positions = np.vstack(positions)
    rotations = np.vstack(rotations)

    return (positions, rotations)
    
    
def analyzer_main():
    home = os.path.expanduser('~')
    path = home + '/.ihmc/logs/perception/'
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

#     for i, file in enumerate(files):
#         print('Index: ', i, ' File: ', file, '\tTitle: ', titles[i])

    # INDEX TO LOAD ----------------------------------------------------->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

    index_to_load = 4

    # INDEX TO LOAD -----------------------------------------------------<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    filename = files[index_to_load]
    filename = '20230308_152625_PerceptionLog.hdf5'
#     print('\nLoading file: ', index_to_load, '\tName: ', filename, '\tTitle: ', titles[index_to_load], '\n')

    data = h5py.File(path + filename, 'r')
    print_file_info(data, filename)

#     output_file = '/home/quantum/Workspace/Code/Resources/IROS_2023/' + titles[index_to_load] + '.txt'

    player_main(data)
    
#     plotter_main(data, output_file)

def kitti_main():
    home = os.path.expanduser('~')
    path = home + '/.ihmc/logs/perception/'

    timestamps_path = '/home/quantum/Workspace/Storage/Other/Temp/dataset/sequences/00/times.txt'
    poses_path = '/home/quantum/Workspace/Storage/Other/Temp/dataset/data_odometry_poses/poses/00.txt'

    dataset_paths = ['/home/quantum/Workspace/Storage/Other/Temp/dataset/sequences/00/image_0/']
    group_names = ['/kitti/left/']

    # data = h5py.File(path + 'KITTI_Dataset_00.hdf5', 'w')

    # data = insert_image_datasets(data, dataset_paths, group_names)
    # data = insert_timestamps(data, timestamps_path, '/kitti/ground_truth/')
    # data = insert_poses(data, poses_path, '/kitti/time/')

    # data.close()

    data = h5py.File(path + 'KITTI_Dataset_00.hdf5', 'r')
    print_file_info(data, 'KITTI_Dataset_00.hdf5')

def plan_view_main(data):

    # Load height map data just as depth data
    # height_map = load_depth(data, 0, 'internal/height/')

    footstep_plan_positions = get_data(data, 'plan/footstep/position/')
    footstep_plan_orientations = get_data(data, 'plan/footstep/orientation/')

    start_positions = get_data(data, 'start/footstep/position/')
    start_orientations = get_data(data, 'start/footstep/orientation/')

    goal_positions = get_data(data, 'goal/footstep/position/')
    goal_orientations = get_data(data, 'goal/footstep/orientation/')

    print("Footstep Plan: ", footstep_plan_positions.shape, footstep_plan_orientations.shape)
    launch_plan_viewer(footstep_plan_positions, footstep_plan_orientations, 
                       start_positions, start_orientations, goal_positions, goal_orientations)

def launch_plan_viewer(footstep_positions, footstep_orientations, 
                       start_positions, start_orientations, goal_positions, goal_orientations):
    

    total_plans = len(data['plan/footstep/position/'].keys())

    # Plot the footstep plan
    for i in range(total_plans):

        height_map = load_depth(data, i, 'cropped/height/')
        height_map = cv2.convertScaleAbs(height_map, alpha=(255.0/65535.0))
        height_map = np.minimum(height_map * 10, 255)

        height_map_display = height_map.copy()

        # convert grayscale to RGB
        height_map_display = cv2.cvtColor(height_map_display, cv2.COLOR_GRAY2RGB)

        print("Start: ", start_positions[i, :], " Goal:", goal_positions[i, :])

        current_plan_positions = footstep_positions[i*10:(i+1)*10, :]
        current_plan_orientations = footstep_orientations[i*10:(i+1)*10, :]

        plot_footstep(height_map_display, start_positions[i, :], start_orientations[i, :], (0, 255, 0), dims=(5,5))
        plot_footstep(height_map_display, goal_positions[i, :], goal_orientations[i, :], (255, 255, 255), dims=(5,5))

        # if current position is not zero, plot footsteps
        plot_footsteps(height_map_display, current_plan_positions, current_plan_orientations)

        # Create a resizeable window and resize by scale factor
        cv2.namedWindow("Footstep Plan", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Footstep Plan", 1000, 1000)
        cv2.imshow("Footstep Plan", height_map_display)
        cv2.waitKey(0)

def plot_footsteps(display, positions, orientations):

    print("Plotting Footsteps. Display Shape: ", display.shape)

    # Plot the footstep plan
    for i in range(10):
        position = positions[i, :]
        orientation = orientations[i, :]

        # set color to red if left foot, yellow if right foot
        color = (0, 0, 255) if i % 2 == 0 else (0, 255, 255)

        # if position is not zero, plot footsteps
        if np.linalg.norm(position) > 0.001:
            plot_footstep(display, position, orientation, color)

def plot_footstep(display, position, orientation, color, dims=(2,4)):
    
    position_on_map = (int(position[1]*50 + display.shape[0]/2), int(position[0]*50 + display.shape[0]/2))
    print("Position on Map: ", position_on_map)

    # Draw a rectangle 2x4 pixels on height map, alternate red and yellow
    display = cv2.rectangle(display, (position_on_map[0] - dims[0], position_on_map[1] - dims[1]), (position_on_map[0] + dims[0], position_on_map[1] + dims[1]), color, -1)



if __name__ == '__main__':

    parser = argparse.ArgumentParser()

    parser.add_argument("--path", help="path to search for files", type=str)
    parser.add_argument("--list", help="increase output verbosity", action="store_true")
    parser.add_argument("--info", help="file name for which to print info", type=str)
    parser.add_argument("--play", help="file name to play", type=str)
    parser.add_argument("--planView", help="file name for height map and footstep plan", type=str)
    parser.add_argument("--plot", help="file name to plot", type=str)
    parser.add_argument("--fix", help="file name to fix", type=str)
    parser.add_argument("--fixAll", help="fixes all files that need fixing in the logs/perception directory", action="store_true")
    parser.add_argument("--dst", help="destination file name", type=str)
    parser.add_argument("--rename", help="rename file to include sensors used", type=str)
    parser.add_argument("--renameAll", help="renames ALL files in the logs/perception directory", action="store_true")

    args = parser.parse_args()
    home = os.path.expanduser('~')
    path = args.path if args.path else home + '/.ihmc/logs/perception/'

    if args.list:
        files = sorted(os.listdir(path))
        print_file_sizes(path, files)

    if args.info:
        data = h5py.File(path + args.info, 'r')
        print_file_info(data, args.info)

    if args.plot:
        data = h5py.File(path + args.plot, 'r')
        plotter_main(data)

    if args.play:
        data = h5py.File(path + args.play, 'r')
        player_main(data)

    if args.fix:
        convert_main(path, args.fix, args.dst)

    if args.fixAll:
        files = sorted(os.listdir(path))
        convert_all(path, files)
    
    if args.rename:
        data = h5py.File(path + args.rename, 'r')
        rename_file(path, data, args.rename)

    if args.renameAll:
        files = sorted(os.listdir(path))
        rename_all_files(path, files)

    if args.planView:
        data = h5py.File(path + args.planView, 'r')
        plan_view_main(data)

    

    # list of good files
    # 20231005_001313_PerceptionLog.hdf5
    # 20231015_183228_PerceptionLog.hdf5
    # 20231010_161751_PerceptionLog.hdf5

    data = h5py.File(path + '20231015_183228_PerceptionLog.hdf5', 'r')
    plan_view_main(data)