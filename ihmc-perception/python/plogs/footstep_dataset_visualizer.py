#! /usr/bin/env python3

import math
import h5py
import numpy as np
import cv2

import os.path
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from plotting.height_map_tools import *
from hdf5_reader import *

def plan_view_main(data):

    # Load height map data just as depth data
    # height_map = load_depth(data, 0, 'internal/height/')
    
    sensor_positions = get_data(data, 'l515/sensor/position/')
    sensor_orientations = get_data(data, 'l515/sensor/orientation/')

    sensor_positions[:, 2] = 0

    footstep_plan_positions = get_data(data, 'plan/footstep/position/')
    footstep_plan_orientations = get_data(data, 'plan/footstep/orientation/')

    start_positions = get_data(data, 'start/footstep/position/')
    start_orientations = get_data(data, 'start/footstep/orientation/')

    goal_positions = get_data(data, 'goal/footstep/position/')
    goal_orientations = get_data(data, 'goal/footstep/orientation/')

    launch_plan_viewer(footstep_plan_positions, footstep_plan_orientations, 
                       start_positions, start_orientations, goal_positions, goal_orientations, sensor_positions, sensor_orientations, n_steps=4)

def visualize_plan(height_map, contact_map, terrain_cost, footstep_plan_poses, start_pose, goal_pose, start_side=0.0, label="Footstep_Plan"):
    
    height_map = cv2.convertScaleAbs(height_map, alpha=(255.0/65535.0))
    height_map = np.minimum(height_map * 10, 255)
    
    # plot_terrain_maps(height_map, contact_map, contact_map)

    height_map_display = height_map.copy()
    height_map_display = cv2.cvtColor(height_map_display, cv2.COLOR_GRAY2RGB)
    height_map_display = cv2.resize(height_map_display, (1000, 1000))

    contact_map = np.stack([contact_map, contact_map, contact_map], axis=2).astype(np.uint8)
    contact_map[:, :, 1] = contact_map[:, :, 0]
    contact_map[:, :, 0] = 0
    contact_map[:, :, 2] = 0
    contact_map = cv2.resize(contact_map, (1000, 1000))


    # print("Height Map Shape:", height_map_display.shape, "Contact Map Shape:", contact_map.shape)    

    # compute scale factor
    scale = 1000 / height_map.shape[0]

    # print("Start pose:", start_pose)
    # print("Goal pose:", goal_pose)

    start_color = (0, 0, 120) if start_side < 0.5 else (0, 120, 120)

    plot_oriented_footstep(height_map_display, start_pose, start_color, scale=scale, dims=(4,8))
    plot_oriented_footstep(height_map_display, goal_pose, (255, 0, 255), scale=scale, dims=(4,8))

    # if current position is not zero, plot footsteps
    plot_oriented_footsteps(height_map_display, footstep_plan_poses, scale)

    stacked_image = np.hstack((height_map_display, contact_map))

    # Create a resizeable window and resize by scale factor
    cv2.namedWindow(label, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(label, 1000, 1000)
    cv2.imshow(label, stacked_image)

    # plot_terrain_maps(height_map, terrain_cost, contact_map)

    code = cv2.waitKeyEx(0)
    # print("Code:", code)

    if code == ord('q'):
        cv2.destroyAllWindows()
        exit()

    return code

def test_visualize(height_map_display, scale=1):
    
    for i in range(10):

        yaw = i*math.pi/10

        # quaternion in the form [w, x, y, z]
        # quaternion = np.array([np.cos(yaw/2), 0, 0, np.sin(yaw/2)])
        # computed_yaw = np.arctan2(2 * (quaternion[1] * quaternion[2] + quaternion[0] * quaternion[3]),
        #             1 - 2 * (quaternion[1]**2 + quaternion[0]**2))

        # quaternion in the form [x, y, z, w]
        quaternion = np.array([0, 0, np.sin(yaw/2), np.cos(yaw/2)])
        computed_yaw = np.arctan2(2 * (quaternion[0] * quaternion[1] + quaternion[3] * quaternion[2]),
                    1 - 2 * (quaternion[0]**2 + quaternion[3]**2))

        pose = np.array([20, i*100, computed_yaw])
        plot_footstep_with_yaw(height_map_display, pose, (0, 255, 0), index=i, scale=scale, dims=(4,8))

def launch_plan_viewer(footstep_positions, footstep_orientations, 
                       start_positions, start_orientations, 
                       goal_positions, goal_orientations,
                       sensor_positions, sensor_orientations, n_steps=4):
    

    total_plans = len(data['plan/footstep/position/'].keys())

    i = 0
    valid = False

    # Plot the footstep plan
    while True:
        height_map = load_depth(data, i, 'cropped/height/')

        sensor_position = sensor_positions[i, :]
        sensor_orientation = sensor_orientations[i, :]

        current_plan_positions = footstep_positions[i*n_steps:(i+1)*n_steps, :]
        current_plan_orientations = footstep_orientations[i*n_steps:(i+1)*n_steps, :]
        
        # count number of non-zero L2 norm positions in current plan
        count_footsteps = np.count_nonzero(np.linalg.norm(current_plan_positions, axis=1))

        valid = not(goal_positions[i, 2] < 0.11 and start_positions[i, 2] < 0.11 or count_footsteps < 3)

        print("Goal Position:", goal_positions[i, :], "Start Position:", start_positions[i, :], "Valid:", valid, "Footsteps:", count_footsteps)
        

        if valid:
            code = visualize_plan(height_map, current_plan_positions - sensor_position, current_plan_orientations, 
                        start_positions[i, :] - sensor_position, start_orientations[i, :], 
                        goal_positions[i, :] - sensor_position, goal_orientations[i, :], i, total_plans)


        if code == 65363 and i < total_plans - 10:
            i += 1
        elif code == 65361 and i > 0:
            i -= 1

        
def plot_oriented_footsteps(display, poses, scale, n_steps=4):

    # Plot the footstep plan
    for i in range(n_steps):
        pose = poses[i, :]
        
        side = i % 2 # 0 for left, 1 for right


        # set color to red if left foot, yellow if right foot
        color = (0, 0, 120) if side < 0.5 else (0, 120, 120)

        # if position is not zero, plot footsteps
        if np.linalg.norm(pose[:2]) > 0.001:
            plot_oriented_footstep(display, pose, color, index=i, scale=scale)

def plot_footstep(display, position, orientation, color, scale=1, dims=(2,4)):
    
    position_on_map = (int(position[1]*50*scale + display.shape[0]/2), int(position[0]*50*scale + display.shape[0]/2))

    # Draw a rectangle 2x4 pixels on height map, alternate red and yellow
    display = cv2.rectangle(display, (position_on_map[0] - dims[0], position_on_map[1] - dims[1]), (position_on_map[0] + dims[0], position_on_map[1] + dims[1]), color, -1)

def plot_oriented_footstep(display, pose, color, index=-1, scale=1, dims=(3,6)):
    position_on_map = (int(pose[1]*50*scale + display.shape[0]/2), int(pose[0]*50*scale + display.shape[0]/2))    
    pose = np.array([position_on_map[0], position_on_map[1], pose[2]])
    plot_footstep_with_yaw(display, pose, color, index=index, scale=scale, dims=dims)

def plot_footstep_with_yaw(display, pose, color, index=-1, scale=1, dims=(3,6)):

    position_on_map = pose[0:2]
    yaw = pose[2]

    # create the footstep rectangle using the position and orientation
    points = np.array([[-dims[0], -dims[1]], [-dims[0], dims[1]], [dims[0], dims[1]], [dims[0], -dims[1]]], dtype=np.float32) * scale

    # create a 2D rotation matrix
    rotation_matrix = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]], dtype=np.float32)

    # rotate the points
    points = np.matmul(rotation_matrix, points.T).T + position_on_map

    # Reshape the array
    points = points.reshape((-1, 1, 2)).astype(np.int32)

    # Draw the polyline on the image
    display = cv2.fillPoly(display, [points], color)

    # plot text on the footstep which is the index of the footstep (size 0.1)
    if index != -1:
        font = cv2.FONT_HERSHEY_SIMPLEX
        text = str(index)
        textsize = cv2.getTextSize(text, font, 1, 2)[0]
        textX = int(position_on_map[0] - textsize[0]/2) + 2
        textY = int(position_on_map[1] + textsize[1]/2)
        cv2.putText(display, text, (textX, textY), font, 0.6, (255, 255, 255), 2)


if __name__ == '__main__':

    # list of good files
    # 20231015_183228_PerceptionLog.hdf5
    # 20231015_234600_PerceptionLog.hdf5
    # 20231016_025456_PerceptionLog.hdf5

    # 20231021_212238_PerceptionLog (Random Start and Goal Poses Within Ranges, Terrain One)

    home = os.path.expanduser('~')
    path = home + '/.ihmc/logs/perception/'

    data = h5py.File(path + '20231022_002815_PerceptionLog.hdf5', 'r')
    plan_view_main(data)