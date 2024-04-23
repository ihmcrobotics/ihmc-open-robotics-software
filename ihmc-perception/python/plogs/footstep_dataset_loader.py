
import numpy as np
import torch
from torch.utils.data import Dataset
from hdf5_reader import *

import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from plotting.height_map_tools import *

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class FootstepDataset(Dataset):
    def __init__(self, data, filename, n_steps=4, flat=False):
        
        self.flat = flat
        self.n_steps = n_steps
        total_height_maps = len(data['cropped/height/'].keys()) - (len(data['cropped/height/'].keys()) % 10)

        self.height_maps = []
        for i in range(total_height_maps):
            height_map_uint16 = load_depth(data, i, 'cropped/height/')


            # subtract the mean height from the height map
            # height_map_uint16 = height_map_uint16 - np.mean(height_map_uint16)

            print("Min: ", np.min(height_map_uint16), "Max: ", np.max(height_map_uint16), "Mean: ", np.mean(height_map_uint16), "StdDev: ", np.std(height_map_uint16))

            height_map_float32 = np.array(height_map_uint16, dtype=np.float32)
            height_map = height_map_float32 / 10000.0
            self.height_maps.append(height_map)

        self.sensor_positions = get_data(data, 'l515/sensor/position/')
        self.sensor_orientations = get_data(data, 'l515/sensor/orientation/')

        self.sensor_positions[:, 2] = 0

        self.footstep_plan_positions = get_data(data, 'plan/footstep/position/')
        self.footstep_plan_orientations = get_data(data, 'plan/footstep/orientation/')

        self.start_side = get_data(data, 'initial/side/')
        self.start_positions = get_data(data, 'start/footstep/position/')
        self.start_orientations = get_data(data, 'start/footstep/orientation/')

        self.goal_positions = get_data(data, 'goal/footstep/position/')
        self.goal_orientations = get_data(data, 'goal/footstep/orientation/')

        total_height_maps = len(self.height_maps)

        self.print_size("File Name: " + filename)

        new_height_maps = []
        new_sensor_positions = []
        new_sensor_orientations = []
        new_footstep_plan_positions = []
        new_footstep_plan_orientations = []
        new_start_positions = []
        new_start_orientations = []
        new_goal_positions = []
        new_goal_orientations = []

        for i in range(total_height_maps):

            current_plan_positions = self.footstep_plan_positions[i*10:i*10 + self.n_steps, :]
            current_plan_orientations = self.footstep_plan_orientations[i*10:i*10 + self.n_steps, :]

            # check if there are no non-zero norm steps in the plan
            count_footsteps = np.count_nonzero(np.linalg.norm(current_plan_positions, axis=1))

            first_step = current_plan_positions[0, :]
            second_step = current_plan_positions[1, :]
            start_step_position = self.start_positions[i, :]

            # cross product start -> first and first -> second 
            # cross_prod = np.cross(first_step - start_step_position, second_step - first_step)

            # # check if start side is 0
            # if cross_prod[2] > 0:
            #     self.start_side[i] = 0

            # valid if has at least 4 non-zero norm steps and start side is 0
            valid = count_footsteps >= n_steps and self.start_side[i] == 1

            if valid:
                new_height_maps.append(self.height_maps[i])
                new_sensor_positions.append(self.sensor_positions[i, :])
                new_sensor_orientations.append(self.sensor_orientations[i, :])
                new_footstep_plan_positions.append(self.footstep_plan_positions[i*10:i*10 + n_steps, :])
                new_footstep_plan_orientations.append(self.footstep_plan_orientations[i*10:i*10 + n_steps, :])
                new_start_positions.append(self.start_positions[i, :])
                new_start_orientations.append(self.start_orientations[i, :])
                new_goal_positions.append(self.goal_positions[i, :])
                new_goal_orientations.append(self.goal_orientations[i, :])

                # get last two non-zero steps
                last_two_step_positions = current_plan_positions[(count_footsteps - 2) : count_footsteps, :]
                last_two_step_orientations = current_plan_orientations[(count_footsteps - 2) : count_footsteps, :]
                
                # set zero footsteps to last two steps in plan
                if count_footsteps < self.n_steps:
                    # print("Shapes: ", count_footsteps, self.n_steps, last_two_step_positions.shape, last_two_step_orientations.shape)
                    self.footstep_plan_positions[count_footsteps:self.n_steps, :] = np.tile(last_two_step_positions, (self.n_steps - count_footsteps, 1))[:self.n_steps - count_footsteps, :]
                    self.footstep_plan_orientations[count_footsteps:self.n_steps, :] = np.tile(last_two_step_orientations, (self.n_steps - count_footsteps, 1))[:self.n_steps - count_footsteps, :]

        self.height_maps = new_height_maps
        self.sensor_positions = np.array(new_sensor_positions)
        self.sensor_orientations = np.array(new_sensor_orientations)
        self.footstep_plan_positions = np.vstack(new_footstep_plan_positions)
        self.footstep_plan_orientations = np.vstack(new_footstep_plan_orientations)
        self.start_positions = np.array(new_start_positions)
        self.start_orientations = np.array(new_start_orientations)
        self.goal_positions = np.array(new_goal_positions)
        self.goal_orientations = np.array(new_goal_orientations)

        self.print_size('After Removal')

        testData = self.__getitem__(1)
        self.image_size = testData[0]
        self.input_size = testData[1]
        self.output_size = testData[2]

    def print_size(self, tag):
        print("Dataset: ------------------------", tag, "-------------------------")
        print(f'Total Height Maps: {len(self.height_maps)}')
        print(f'Total Sensor Positions: {self.sensor_positions.shape}')
        print(f'Total Sensor Orientations: {self.sensor_orientations.shape}')
        print(f'Total Footstep Plan Positions: {self.footstep_plan_positions.shape}')
        print(f'Total Footstep Plan Orientations: {self.footstep_plan_orientations.shape}')
        print(f'Total Start Positions: {self.start_positions.shape}')
        print(f'Total Start Orientations: {self.start_orientations.shape}')
        print(f'Total Goal Positions: {self.goal_positions.shape}')
        print(f'Total Goal Orientations: {self.goal_orientations.shape}')

        # print min, max, mean and stddev for first height map
        print(f'Height Map: Min: {np.min(self.height_maps[0])}, Max: {np.max(self.height_maps[0])}, Mean: {np.mean(self.height_maps[0])}, StdDev: {np.std(self.height_maps[0])}')


    def __getitem__(self, index):

        # convert quaternion to yaw
        sensor_quaternion = self.sensor_orientations[index, :]
        sensor_yaw = np.arctan2(2 * (sensor_quaternion[0] * sensor_quaternion[1] + sensor_quaternion[3] * sensor_quaternion[2]),
                    1 - 2 * (sensor_quaternion[0]**2 + sensor_quaternion[3]**2))
        sensor_pose = np.array([self.sensor_positions[index, 0], self.sensor_positions[index, 1], sensor_yaw], dtype=np.float32)
        
        start_side = self.start_side[index]

        start_quaternion = self.start_orientations[index, :]
        start_yaw = np.arctan2(2 * (start_quaternion[0] * start_quaternion[1] + start_quaternion[3] * start_quaternion[2]),
                    1 - 2 * (start_quaternion[0]**2 + start_quaternion[3]**2))
        start_pose = np.array([self.start_positions[index, 0] - sensor_pose[0], self.start_positions[index, 1] - sensor_pose[1], start_yaw], dtype=np.float32)

        goal_quaternion = self.goal_orientations[index, :]
        goal_yaw = np.arctan2(2 * (goal_quaternion[0] * goal_quaternion[1] + goal_quaternion[3] * goal_quaternion[2]),
                    1 - 2 * (goal_quaternion[0]**2 + goal_quaternion[3]**2))
        goal_pose = np.array([self.goal_positions[index, 0] - sensor_pose[0], self.goal_positions[index, 1] - sensor_pose[1], goal_yaw])

        footstep_plan_quaternions = self.footstep_plan_orientations[index * self.n_steps:index * self.n_steps + self.n_steps, :]
        footstep_plan_yaws = np.arctan2(2 * (footstep_plan_quaternions[:, 0] * footstep_plan_quaternions[:, 1] + footstep_plan_quaternions[:, 3] * footstep_plan_quaternions[:, 2]),
                    1 - 2 * (footstep_plan_quaternions[:, 0]**2 + footstep_plan_quaternions[:, 3]**2))
        
        footstep_plan_poses = self.footstep_plan_positions[index * self.n_steps:index * self.n_steps + self.n_steps, :] - sensor_pose
        footstep_plan_poses[:, 2] = footstep_plan_yaws
        footstep_plan_poses = np.array([footstep_plan_poses], dtype=np.float32)

        terrain_cost_map = compute_terrain_cost_map(self.height_maps[index])
        contact_map = compute_contact_map(terrain_cost_map)


        # Inputs
        height_map_input = torch.Tensor(self.height_maps[index]).unsqueeze(0).to(device)
        terrian_cost_input = torch.Tensor(terrain_cost_map).unsqueeze(0).to(device)
        contact_map_input = torch.Tensor(contact_map).unsqueeze(0).to(device)
        
        if self.flat: 
            # stack input map with height map and add another dimension
            input_map = np.zeros_like(self.height_maps[index])
            start_indices = self.start_positions[index] * 50 + 100
            goal_indices = self.goal_positions[index] * 50 + 100
            input_map[int(start_indices[0]), int(start_indices[1])] = 1
            input_map[int(goal_indices[0]), int(goal_indices[1])] = 1

            # print("Start: ", start_indices, "Goal: ", goal_indices)

            input_map = torch.Tensor(input_map).unsqueeze(0).to(device)

            height_map_input = torch.cat((height_map_input, input_map), dim=0)

        start_pose = torch.Tensor(start_pose).to(device)
        goal_pose = torch.Tensor(goal_pose).to(device)
        linear_input = torch.cat((start_pose, goal_pose), dim=0) 

        # Outputs
        footstep_plan_poses = torch.Tensor(footstep_plan_poses).to(device)
        linear_output = torch.flatten(footstep_plan_poses)

        return height_map_input, linear_input, linear_output, contact_map_input, terrian_cost_input
        
    def __len__(self) -> int:
        return len(self.height_maps)

