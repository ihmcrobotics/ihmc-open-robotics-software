import torch
torch.cuda.empty_cache()
import torch.nn.functional as F
import torch.nn as nn
from torch.nn import Module, Sequential, Linear, ReLU, Dropout, BatchNorm1d
from torch.utils.data import Dataset, DataLoader
from torch.utils.tensorboard import SummaryWriter
from tqdm import tqdm
import os.path
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from plotting.height_map_tools import *
from footstep_dataset_loader import visualize_plan
from hdf5_reader import *
import os

# create torch tensors on GPU
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
writer = SummaryWriter("runs/footstep_predictor")


class FootstepDataset(Dataset):
    def __init__(self, data, filename, n_steps=4):

        self.n_steps = n_steps
        total_height_maps = len(data['cropped/height/'].keys()) - (len(data['cropped/height/'].keys()) % 10)

        self.height_maps = []
        for i in range(total_height_maps):
            height_map_uint16 = load_depth(data, i, 'cropped/height/')
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
            cross_prod = np.cross(first_step - start_step_position, second_step - first_step)

            # check if start side is 0
            if cross_prod[2] > 0:
                self.start_side[i] = 0

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


    def __getitem__(self, index):

        # Use the following to compute the yaw:
        # computed_yaw = np.arctan2(2 * (quaternion[0] * quaternion[1] + quaternion[3] * quaternion[2]),
        # 1 - 2 * (quaternion[0]**2 + quaternion[3]**2))

        # convert quaternion to yaw
        sensor_quaternion = self.sensor_orientations[index, :]
        sensor_yaw = np.arctan2(2 * (sensor_quaternion[0] * sensor_quaternion[1] + sensor_quaternion[3] * sensor_quaternion[2]),
                                1 - 2 * (sensor_quaternion[0]**2 + sensor_quaternion[3]**2))
        sensor_pose = np.array([self.sensor_positions[index, 0], self.sensor_positions[index, 1], sensor_yaw], dtype=np.float32)

        # get intial side
        start_side = self.start_side[index]

        # convert quaternion to yaw
        start_quaternion = self.start_orientations[index, :]
        start_yaw = np.arctan2(2 * (start_quaternion[0] * start_quaternion[1] + start_quaternion[3] * start_quaternion[2]),
                               1 - 2 * (start_quaternion[0]**2 + start_quaternion[3]**2))
        start_pose = np.array([self.start_positions[index, 0] - sensor_pose[0], self.start_positions[index, 1] - sensor_pose[1], start_yaw], dtype=np.float32)

        goal_quaternion = self.goal_orientations[index, :]
        goal_yaw = np.arctan2(2 * (goal_quaternion[0] * goal_quaternion[1] + goal_quaternion[3] * goal_quaternion[2]),
                              1 - 2 * (goal_quaternion[0]**2 + goal_quaternion[3]**2))
        goal_pose = np.array([self.goal_positions[index, 0] - sensor_pose[0], self.goal_positions[index, 1] - sensor_pose[1], goal_yaw])

        # convert quaternion to yaw
        footstep_plan_quaternions = self.footstep_plan_orientations[index*n_steps:index*n_steps + self.n_steps, :]
        footstep_plan_yaws = np.arctan2(2 * (footstep_plan_quaternions[:, 0] * footstep_plan_quaternions[:, 1] + footstep_plan_quaternions[:, 3] * footstep_plan_quaternions[:, 2]),
                                        1 - 2 * (footstep_plan_quaternions[:, 0]**2 + footstep_plan_quaternions[:, 3]**2))

        footstep_plan_poses = self.footstep_plan_positions[index*n_steps:index*n_steps + self.n_steps, :] - sensor_pose
        footstep_plan_poses[:, 2] = footstep_plan_yaws
        footstep_plan_poses = np.array([footstep_plan_poses], dtype=np.float32)

        terrain_cost_map = compute_terrain_cost_map(self.height_maps[index])
        contact_map = compute_contact_map(terrain_cost_map)

        # add contact map and terrain cost map as subplots on imshow

        # plt.subplot(1, 3, 1)
        # plt.imshow(terrain_cost_map, cmap='gray')
        # plt.title('Terrain Cost Map')
        # plt.subplot(1, 3, 2)
        # plt.imshow(contact_map, cmap='gray')
        # plt.title('Contact Map')
        # plt.subplot(1, 3, 3)
        # plt.imshow(self.height_maps[index], cmap='gray')

        # plt.show()

        # Inputs
        # --------- Image Inputs (float54)
        height_map_input = torch.Tensor(self.height_maps[index]).unsqueeze(0).to(device)
        terrian_cost_input = torch.Tensor(terrain_cost_map).unsqueeze(0).to(device)
        contact_map_input = torch.Tensor(contact_map).unsqueeze(0).to(device)

        # --------- Pose Inputs (X, Y, Yaw) with float 64
        start_pose = torch.Tensor(start_pose).to(device)
        goal_pose = torch.Tensor(goal_pose).to(device)
        linear_input = torch.cat((start_pose, goal_pose), dim=0)

        # Outputs
        # --------- 3D Pose Outputs
        footstep_plan_poses = torch.Tensor(footstep_plan_poses).to(device)
        linear_output = torch.flatten(footstep_plan_poses)

        # print("Goal Pose: ", goal_pose)

        return height_map_input, linear_input, linear_output, contact_map_input, terrian_cost_input

    def __len__(self) -> int:
        return len(self.height_maps)

class FootstepPredictor(nn.Module):
    def __init__(self, input_size, output_size):
        super(FootstepPredictor, self).__init__()

        # Convolutional layers for the image input
        self.conv1 = nn.Conv2d(1, 32, kernel_size=3, stride=1, padding=1)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1)
        self.conv3 = nn.Conv2d(64, 128, kernel_size=3, stride=1, padding=1)
        self.conv4 = nn.Conv2d(128, 256, kernel_size=3, stride=1, padding=1)

        # Pooling layer
        self.maxpool = nn.MaxPool2d(kernel_size=2, stride=2)

        # Fully connected layers for image features
        self.fc_image = nn.Linear(256 * 12 * 12, 512)

        # Fully connected layers for pose features
        self.fc_pose = nn.Linear(input_size, 512)

        # Batch normalization layers
        self.bn_image = nn.BatchNorm1d(512)
        self.bn_pose = nn.BatchNorm1d(512)

        # Dropout layer
        self.dropout = nn.Dropout(0.5)

        # Output layer
        self.fc_out = nn.Linear(1024, output_size)

    def forward(self, image, pose):
        # Forward pass for image input
        x = self.conv1(image)
        x = F.relu(x)
        x = self.maxpool(x)
        x = self.conv2(x)
        x = F.relu(x)
        x = self.maxpool(x)
        x = self.conv3(x)
        x = F.relu(x)
        x = self.maxpool(x)
        x = self.conv4(x)
        x = F.relu(x)
        x = self.maxpool(x)

        x = torch.flatten(x, 1)
        x = self.fc_image(x)
        x = self.bn_image(x)
        x = F.relu(x)

        # Forward pass for pose input
        pose = self.fc_pose(pose)
        pose = self.bn_pose(pose)
        pose = F.relu(pose)

        # Concatenate image and pose features
        x = torch.cat((x, pose), dim=1)

        # Apply dropout
        x = self.dropout(x)

        # Output layer
        x = self.fc_out(x)

        return x

def train_store(train_dataset, val_dataset, batch_size, epochs, criterion, model_path, warm_start=False):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    train_loader = DataLoader(dataset=train_dataset, batch_size=batch_size, shuffle=True, num_workers=0)
    val_loader = DataLoader(dataset=val_dataset, batch_size=batch_size, shuffle=False, num_workers=0)
    input_size = train_dataset[0][1].shape[0]
    output_size = train_dataset[0][2].shape[0]
    model = FootstepPredictor(input_size, output_size).to(device)

    if warm_start:
        # Load weights from previously trained model
        model_files = sorted([name for name in os.listdir(model_path) if name.endswith('.pt')])
        if len(model_files) > 0:
            print("Loading Model: ", model_files[-1])
            model.load_state_dict(torch.load(model_path + model_files[-1]))

    optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)  # Adjust learning rate if needed

    for epoch in range(epochs):
        loop = tqdm(train_loader, bar_format='{l_bar}{bar:30}{r_bar}{bar:-30b}')

        running_loss = 0
        for i, (image, pose, target, _, _) in enumerate(loop):
            image, pose, target = image.to(device), pose.to(device), target.to(device)

            optimizer.zero_grad()
            output = model(image, pose)
            loss = criterion(output, target)
            loss.backward()
            optimizer.step()

            running_loss += loss.item()
            average_loss = running_loss / (i + 1)
            loop.set_description(f'Epoch [{epoch+1}/{epochs}]')
            loop.set_postfix(loss=average_loss)

        print("Epoch: ", epoch, "Training Loss: ", average_loss, end="\t")

        total_validation_loss = 0
        for image, pose, target, _, _ in val_loader:
            image, pose, target = image.to(device), pose.to(device), target.to(device)
            model.eval()
            with torch.no_grad():
                output = model(image, pose)
                loss = criterion(output, target)
                total_validation_loss += loss.item()

        average_validation_loss = total_validation_loss / len(val_loader)

def footstep_loss(output, target, contact_map):

    # Reshape the output tensor to extract footstep positions (8, 1, 12)

    print("Output Shape: ", output.shape)

    output = output.view(-1, 3, 4)
    output_reshaped = output.view(-1, 4, 4)

    # Extract x and y coordinates of predicted footstep positions
    fx = output_reshaped[:, :, 0]
    fy = output_reshaped[:, :, 1]

    # Convert footstep positions to pixel indices and clamp within valid range
    fx = (fx * 50 + 100).clamp(0, 199).long()
    fy = (fy * 50 + 100).clamp(0, 199).long()

    # Compute contact scores from the contact map at predicted footstep positions
    contact_scores = contact_map[:, 0, fx, fy].sum(dim=1) / 4.0

    # Calculate contact loss as the difference between predicted and actual contact scores
    contact_loss = (1.0 - contact_scores) * 100.0

    # Compute L1 loss between predicted and target footstep positions
    l1_loss = torch.nn.L1Loss()(output, target)

    # Combine L1 loss and contact map loss
    total_loss = l1_loss + contact_loss

    return total_loss


def load_validate(val_dataset, batch_size, model_path):
    loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=True)

    # Load the model
    input_size = val_dataset[0][1].shape[0]
    output_size = val_dataset[0][2].shape[0]
    model = FootstepPredictor(input_size, output_size)

    # Load the latest model file
    model_files = sorted([name for name in os.listdir(model_path) if name.endswith('.pt')])
    if model_files:
        model_file = model_files[-1]
        print("Loading Model: ", model_file)
        model.load_state_dict(torch.load(os.path.join(model_path, model_file)))
    else:
        print("No model files found.")
        return

    model.eval()
    model.to(device)

    with torch.no_grad():
        for i, (height_map_input, linear_input, target_output, contact_map, terrain_cost) in enumerate(loader):
            height_map_input = height_map_input.to(device)
            linear_input = linear_input.to(device)
            target_output = target_output.to(device)
            contact_map = contact_map.to(device)

            predict_output = model(height_map_input, linear_input)

            # Get only the first 4 steps
            predict_output = predict_output[:3 * n_steps, :].reshape((n_steps, 3))
            target_output = target_output[:3 * n_steps].reshape((n_steps, 3))

            # Compute loss
            loss = print_loss(predict_output, target_output, contact_map)

            visualize_output(height_map_input, linear_input, predict_output, contact_map, terrain_cost, label="Prediction")
            visualize_output(height_map_input, linear_input, target_output, contact_map, terrain_cost, label="Target")

def visualize_output(height_map_input, linear_input, final_output, contact_map_uint8, terrain_cost, n_steps=4, label="Footstep Plan"):
    # Convert tensors to numpy arrays
    output = final_output.cpu().numpy().squeeze()
    linear_input = linear_input.cpu().numpy().squeeze()
    height_map_uint16 = (height_map_input.cpu().numpy() * 10000.0).astype(np.uint16).reshape((201, 201))
    terrain_cost = terrain_cost.cpu().numpy().astype(np.uint8).reshape((201, 201))
    contact_map_uint8 = contact_map_uint8.cpu().numpy().astype(np.uint8).reshape((201, 201))

    start_pose = linear_input[:3]
    goal_pose = linear_input[3:6]
    plan_poses = output[:3 * n_steps].reshape((n_steps, 3))[:, :3]

    # Visualize plan
    visualize_plan(height_map_uint16, contact_map_uint8, terrain_cost, plan_poses, start_pose, goal_pose, label=label)

def load_dataset(validation_split):
    home = os.path.expanduser('~')
    path = os.path.join(home, 'Downloads', 'Planning_Datasets', 'Basic')

    # List input files
    files = sorted([file for file in os.listdir(path) if file.endswith('.hdf5')])
    labels = ['MCFP', 'AStar']
    files = [file for file in files if any(label in file for label in labels)]
    files = [files[-1]]

    datasets = []
    for file in files:
        print("Loading File: ", file)
        data = h5py.File(os.path.join(path, file), 'r')
        dataset = FootstepDataset(data, file, n_steps=n_steps)
        datasets.append(dataset)

    dataset = torch.utils.data.ConcatDataset(datasets)
    val_size = int(validation_split * len(dataset))
    train_size = len(dataset) - val_size
    train_dataset, val_dataset = torch.utils.data.random_split(dataset, [train_size, val_size])

    return train_dataset, val_dataset

def visualize_dataset(dataset, batch_size=1):
    loader = DataLoader(dataset, batch_size=batch_size, shuffle=True)
    for i, (height_map_input, linear_input, target_output, contact_map, terrain_cost) in enumerate(loader):
        visualize_output(height_map_input, linear_input, target_output, contact_map, terrain_cost)

import os
import argparse
import torch

if __name__ == "__main__":
    home = os.path.expanduser('~')
    model_path = os.path.join(home, 'Downloads', 'Model_Weights')
    datasets_path = os.path.join(home, 'Downloads', 'Planning_Datasets', 'Basic')
    total_files = 1

    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Footstep Prediction Trainer')
    parser.add_argument('--train', action='store_true', help='Train the model')
    parser.add_argument('--files', type=int, help='Total Files to Load')
    parser.add_argument('--raw', action='store_true', help='Raw Visualization')
    args = parser.parse_args()

    if args.files:
        total_files = args.files

    train = args.train
    n_steps = 4
    batch_size = 8

    # Load dataset
    train_dataset, val_dataset = load_dataset(validation_split=0.1)

    visualize_raw = args.raw

    # Visualize raw data if specified
    if visualize_raw:
        visualize_dataset(train_dataset)
        exit()

    # Train the model if specified, otherwise load and validate
    if train:
        criterion = footstep_loss
        train_store(train_dataset, val_dataset, batch_size=batch_size, epochs=30, criterion=criterion, model_path=model_path)
    else:
        load_validate(train_dataset, batch_size=1, model_path=model_path)

    # Clear CUDA cache
    torch.cuda.empty_cache()
