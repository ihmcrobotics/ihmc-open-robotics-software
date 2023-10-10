#! /usr/bin/env python3

import h5py
import numpy as np
import cv2

from hdf5_reader import *

def plan_view_main(data):

    # Load height map data just as depth data
    # height_map = load_depth(data, 0, 'internal/height/')

    footstep_plan_positions = get_data(data, 'plan/footstep/position/')
    footstep_plan_orientations = get_data(data, 'plan/footstep/orientation/')

    start_positions = get_data(data, 'start/footstep/position/')
    start_orientations = get_data(data, 'start/footstep/orientation/')

    goal_positions = get_data(data, 'goal/footstep/position/')
    goal_orientations = get_data(data, 'goal/footstep/orientation/')

    launch_plan_viewer(footstep_plan_positions, footstep_plan_orientations, 
                       start_positions, start_orientations, goal_positions, goal_orientations)

def launch_plan_viewer(footstep_positions, footstep_orientations, 
                       start_positions, start_orientations, goal_positions, goal_orientations):
    

    total_plans = len(data['plan/footstep/position/'].keys())

    i = 0

    # Plot the footstep plan
    while True:

        height_map = load_depth(data, i, 'cropped/height/')
        height_map = cv2.convertScaleAbs(height_map, alpha=(255.0/65535.0))
        height_map = np.minimum(height_map * 10, 255)

        height_map_display = height_map.copy()

        # convert grayscale to RGB
        height_map_display = cv2.cvtColor(height_map_display, cv2.COLOR_GRAY2RGB)

        sensor_positions = get_data(data, 'l515/sensor/position/')
        sensor_orientations = get_data(data, 'l515/sensor/orientation/')

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
        code = cv2.waitKeyEx(0)

        if code == ord('q'):
            break
        elif code == 65363 and i < total_plans - 1:
            i += 1
        elif code == 65361 and i > 0:
            i -= 1
        
        print("Code: ", code)

def plot_footsteps(display, positions, orientations):

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

    # Draw a rectangle 2x4 pixels on height map, alternate red and yellow
    display = cv2.rectangle(display, (position_on_map[0] - dims[0], position_on_map[1] - dims[1]), (position_on_map[0] + dims[0], position_on_map[1] + dims[1]), color, -1)



if __name__ == '__main__':

    # list of good files
    # 20231005_001313_PerceptionLog.hdf5

    path = '/home/bmishra/.ihmc/logs/perception/'

    data = h5py.File(path + '20231010_161751_PerceptionLog.hdf5', 'r')
    plan_view_main(data)