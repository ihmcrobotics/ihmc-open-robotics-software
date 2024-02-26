import torch
import gc
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


class FootstepPredictor(Module):
    def __init__(self, input_size, output_size):
        super(FootstepPredictor, self).__init__()

        print("FootstepPredictor (Model) -> Linear Input Size: ", input_size, "Linear Output Size: ", output_size)

        # convolutional layers given a 200x200 16-bit grayscale image
        self.conv2d_1 = torch.nn.Conv2d(1, 48, kernel_size=3, stride=1, padding=1)
        self.conv2d_2 = torch.nn.Conv2d(48, 96, kernel_size=3, stride=1, padding=1)
        self.conv2d_3 = torch.nn.Conv2d(96, 48, kernel_size=3, stride=1, padding=1)
        self.conv2d_4 = torch.nn.Conv2d(48, 32, kernel_size=3, stride=1, padding=1)

        self.maxpool2d_22 = torch.nn.MaxPool2d(kernel_size=2, stride=2, padding=0)
        self.maxpool2d_44 = torch.nn.MaxPool2d(kernel_size=4, stride=4, padding=0)

        self.hfc0 = torch.nn.Linear(32 * 12 * 12, 4096)
        self.hbn0 = torch.nn.BatchNorm1d(4096)
        self.hfc1 = torch.nn.Linear(4096, 1024)
        self.hbn1 = torch.nn.BatchNorm1d(1024)

        self.fc0 = torch.nn.Linear(input_size, 1024)
        self.bn0 = torch.nn.BatchNorm1d(1024)
        self.dropout0 = torch.nn.Dropout(0.1)

        self.fc1 = torch.nn.Linear(1024 + 1024, 1024)
        self.bn1 = torch.nn.BatchNorm1d(1024)
        self.dropout1 = torch.nn.Dropout(0.05)
        
        self.fc2 = torch.nn.Linear(1024, 1024)
        self.bn2 = torch.nn.BatchNorm1d(1024)
        self.fc3 = torch.nn.Linear(1024, 512)
        self.bn3 = torch.nn.BatchNorm1d(512)
        self.fc4 = torch.nn.Linear(512, 256)
        self.bn4 = torch.nn.BatchNorm1d(256)
        self.fc5 = torch.nn.Linear(256, 128)
        self.bn5 = torch.nn.BatchNorm1d(128)
        self.fc6 = torch.nn.Linear(128, output_size)

    def forward(self, h1, l1):

        # print("\n\nForward Pass Input Shape: ", h1.shape, l1.shape, end=" ")

        # x1 is image 200x200 16-bit grayscale and x2 is the 3D pose
        h1 = self.conv2d_1(h1)
        h1 = self.maxpool2d_22(h1)
        h1 = self.conv2d_2(h1)
        h1 = self.maxpool2d_22(h1)
        h1 = self.conv2d_3(h1)
        h1 = self.maxpool2d_22(h1)
        h1 = self.conv2d_4(h1)
        h1 = self.maxpool2d_22(h1)


        h1 = torch.flatten(h1, 1)
        h1 = self.hfc0(h1)
        h1 = self.hbn0(h1)
        h1 = F.leaky_relu(h1)
        h1 = self.hfc1(h1)
        h1 = self.hbn1(h1)
        h1 = F.leaky_relu(h1)

        l1 = self.fc0(l1)        
        l1 = self.bn0(l1)
        l1 = F.leaky_relu(l1)
        
        # print shape, mean, min, max and stddev
        print(
            # "Shapes: ", h1.shape, l1.shape,
            "Mean: ", round(h1.mean().item(), 3), round(l1.mean().item(), 3), 
            "Min: ", round(h1.min().item(), 3), round(l1.min().item(), 3), 
            "Max: ", round(h1.max().item(), 3), round(l1.max().item(), 3), 
            "Stddev: ", round(h1.std().item(), 3), round(l1.std().item(), 3)
        )

        # flatten x1 and concatenate with x2
        x = torch.cat((h1, l1), dim=1)

        # fully connected layers
        x = self.fc1(x)
        x = self.bn1(x)
        x = F.leaky_relu(x)
        x = self.dropout1(x)

        x = self.fc2(x)
        x = self.bn2(x)
        x = F.leaky_relu(x)
        x = self.dropout1(x)

        x = self.fc3(x)
        x = self.bn3(x)
        x = F.leaky_relu(x)
        x = self.dropout1(x)

        x = self.fc4(x)
        x = self.bn4(x)
        x = F.leaky_relu(x)
        x = self.dropout1(x)

        x = self.fc5(x)
        x = self.bn5(x)
        x = F.leaky_relu(x)
        x = self.dropout1(x)

        x = self.fc6(x)

        # print("Forward Pass Output Shape: ", x.shape)

        return x

def train_store(train_dataset, val_dataset, batch_size, epochs, criterion, model_path, warm_start=False):
    
    train_loader = DataLoader(dataset=train_dataset, batch_size=batch_size, shuffle=True, num_workers=0)
    val_loader = DataLoader(dataset=val_dataset, batch_size=batch_size, shuffle=False, num_workers=0)
    input_size = train_dataset[0][1].shape[0]
    output_size = train_dataset[0][2].shape[0]
    model = FootstepPredictor(input_size, output_size).to(device)

    if warm_start:
        # load weights from previously trained model to start from where it left off
        model_files = sorted([name for name in os.listdir(model_path) if name.endswith('.pt')])
        if len(model_files) > 0:
            print("Loading Model: ", model_files[-1])
            model.load_state_dict(torch.load(model_path + model_files[-1]))

    # define optimizer
    # optimizer = torch.optim.Adam(model.parameters(), lr=1e-3, weight_decay = 1e-8)
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-6)

    # train the model
    for epoch in range(epochs):
        loop = tqdm(train_loader, bar_format='{l_bar}{bar:30}{r_bar}{bar:-30b}')

        running_loss = 0
        for i, (x1, x2, y, cm, tc) in enumerate(loop):

            if x1.shape[0] > 1:
                x1 = x1.to(device)
                x2 = x2.to(device)
                y = y.to(device)
                cm = cm.to(device)

                optimizer.zero_grad()
                y_pred = model(x1, x2)
                loss = criterion(y_pred, y, cm)

                loss.backward()
                optimizer.step()

                # running_loss+=loss.item()

                average_loss = running_loss/(y.size(0)*(i+1))
                loop.set_description(f'Epoch [{epoch+1}/{epochs}]')
                loop.set_postfix(loss=average_loss)
                # writer.add_scalar('training loss', average_loss, epoch * len(train_loader) + i)

                running_loss += loss.item()

        average_training_loss = running_loss / len(train_loader)
        print("Epoch: ", epoch, "Training Loss: ", average_training_loss, end="\t")

        total_validation_loss = 0
        for x1, x2, y, cm, tc in val_loader:
            # send data to gpu
            if x1.shape[0] > 1:
                x1 = x1.to(device)
                x2 = x2.to(device)
                y = y.to(device)
                model.eval()
                y_pred = model(x1, x2)
                loss = criterion(y_pred, y, cm)
                valid_loss = loss.item()
                total_validation_loss += valid_loss
        
        average_validation_loss = total_validation_loss / len(val_loader)
        print("Validation Set: ", len(val_loader), "\tValidiation loss: ", average_validation_loss)
        # writer.add_scalar('validation loss', valid_loss, epoch * len(train_loader) + i)

    # save the model
    ckpt_count = len([name for name in os.listdir(model_path) if name.endswith('.pt')])
    file_name = 'footstep_predictor'
    torch.save(model.state_dict(), model_path + file_name + '.pt')
    torch.onnx.export(model, (x1[0].unsqueeze(0), x2[0].unsqueeze(0)), model_path + file_name + '.onnx', verbose=False)

    print("Saved as: ", file_name + '.pt')
    # writer.close()


def load_validate(val_dataset, batch_size, model_path):
    loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=True)
    
    # Load the model
    input_size = val_dataset[0][1].shape[0]
    output_size = val_dataset[0][2].shape[0]
    model = FootstepPredictor(input_size, output_size)

    # those that end in .pt
    model_files = sorted([name for name in os.listdir(model_path) if name.endswith('.pt')])

    print("Loading Model: ", model_files[-1])

    model.load_state_dict(torch.load(model_path + "footstep_predictor.pt"))
    model.eval()
    model.to(device)

    with torch.no_grad():
        for i, (height_map_input, linear_input, target_output, contact_map, terrain_cost) in enumerate(loader):

            height_map_input = height_map_input.to(device)
            linear_input = linear_input.to(device)
            target_output = target_output.to(device)
            contact_map = contact_map.to(device)

            predict_output = model(height_map_input, linear_input)

            # get only the first 4 steps
            predict_output = predict_output[0:3*n_steps, :].reshape((n_steps, 3))
            target_output = target_output[0:3*n_steps].reshape((n_steps, 3))

            # compute loss
            print_loss(predict_output, target_output, contact_map)

            visualize_output(height_map_input, linear_input, predict_output, contact_map, terrain_cost, label="Prediction")
            visualize_output(height_map_input, linear_input, target_output, contact_map, terrain_cost, label="Target")

            
def visualize_output(height_map_input, linear_input, final_output, contact_map_uint8, terrain_cost, n_steps=4, label="Footstep Plan"):

    output = final_output.cpu().numpy()
    output = output.squeeze()
    # print("Predict Output", output.shape)

    linear_input = linear_input.cpu().numpy()
    linear_input = linear_input.squeeze()
    # print("Linear Input", linear_input.shape)

    # convert height map to numpy array 16-bit grayscale
    height_map_uint16 = np.array(height_map_input.cpu().numpy() * 10000.0, dtype=np.uint16)

    # reshape for opencv
    height_map_uint16 = height_map_uint16.reshape((201, 201))        

    terrain_cost = np.array(terrain_cost.cpu().numpy(), dtype=np.uint8)
    terrain_cost = terrain_cost.reshape((201, 201))

    # convert contact map to numpy array 8-bit grayscale
    contact_map_uint8 = np.array(contact_map_uint8.cpu().numpy(), dtype=np.uint8)
    contact_map_uint8 = contact_map_uint8.reshape((201, 201))

    start_pose = linear_input[0:3]
    goal_pose = linear_input[3:6]
    plan_poses = output[0:3*n_steps].reshape((n_steps, 3))[:, 0:3]

    # print("Height Map: ", height_map.tolist())

    # visualize plan
    visualize_plan(height_map_uint16, contact_map_uint8, terrain_cost, plan_poses, 
                    start_pose, goal_pose, label=label)

def load_dataset(validation_split):
    home = os.path.expanduser('~')
    path = home + '/Downloads/Planning_Datasets/Basic/'
    
    # input_files = ['20231018_135001_PerceptionLog.hdf5', 
    #                     '20231018_143108_PerceptionLog.hdf5']
    
    files = sorted(os.listdir(path))
    files = [file for file in files if ".hdf5" in file]

    labels = [
                'MCFP', 
                'AStar'
    ]

    # filter by label
    files = [file for file in files if any(label in file for label in labels)]
    

    files = [files[-1]]

    
    datasets = []

    for file in files:

        print("Loading File: ", file)

        data = h5py.File(path + file, 'r')
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

        height_map_input = height_map_input.to(device)
        linear_input = linear_input.to(device)
        target_output = target_output.to(device)


        visualize_output(height_map_input, linear_input, target_output, contact_map, terrain_cost)

def footstep_loss(output, target, contact_map):
    
    return torch.nn.L1Loss()(output, target)

    # print("Output Shape: ", output.shape, "Target Shape: ", target.shape, "Contact Map Shape: ", contact_map.shape)

    # output_reshaped = torch.reshape(output, (-1, 4, 3))
    # fx = output_reshaped[:, :, 0]
    # fy = output_reshaped[:, :, 1]
    # contact = contact_map[:, 0, fx.long(), fy.long()]  # Fix: Use contact_map provided in batch
    # contact_loss = -contact

    # sum the contact map values at the predicted footstep locations and use it as a loss

    #take all rows but only 1st, 4th, 7th, 10th columns

    fx = (output[:,[0, 3, 6, 9]])
    fx = (fx * 50 + 100)
    fy = (output[:,[1, 4, 7, 10]])
    fy = (fy * 50 + 100)

    # put limits on the indices
    fx = torch.clamp(fx, 0, 199)
    fy = torch.clamp(fy, 0, 199)

    # cast to long
    fx = fx.long()
    fy = fy.long()

    sum_loss = []
    for itr in range(len(output)):
        per_image_score = 0
        for itr2 in range(n_steps):
            per_image_score += contact_map[itr, 0, fx[itr, itr2].item(), fy[itr, itr2].item()].item()
        
        # print("Per Image Score: ", per_image_score)
        
        total_contact_score = per_image_score / n_steps
        contact_loss = (1.0 - total_contact_score) * 100.0
        curr_output = output[itr].unsqueeze(0)
        curr_target = target[itr].unsqueeze(0)
        l1_loss = torch.nn.L1Loss()(curr_output, curr_target)

        # print("L1 Loss: ", l1_loss.item(), "Contact Loss: ", contact_loss)

        sum_loss.append((l1_loss + contact_loss))
    
    # return torch.nn.L1Loss()(output, target)

    # 
    # print('here',sum(sum_loss) / len (sum_loss))
    return sum(sum_loss) / len (sum_loss)


    # print("FX: ", fx)
    # print("FY: ", fy)


    # contact_vector = contact_map[fx, fy]
    # print("Unique: ", torch.unique(contact_vector))

    # total_contact_score = torch.sum(contact_vector) / n_steps
    # contact_loss = 1.0 - total_contact_score

    # print("Contact Vector: ", contact_vector)
    # print("Total Contact Score: ", total_contact_score.item())

    # l1_loss = torch.nn.L1Loss()(output, target)

    # return (l1_loss + contact_loss) / 2.0

def print_loss(output, target, contact_map):
    
    # print("Shapes: ", output.shape, target.shape, contact_map.shape)

    # print min and max for contact map
    # print("Contact Map Min: ", torch.min(contact_map), "Contact Map Max: ", torch.max(contact_map))

    # sum the contact map values at the predicted footstep locations and use it as a loss

    # sum the contact map values at the predicted footstep locations and use it as a loss
    fx = (output[:, 0] * 50 + 100)
    fy = (output[:, 1] * 50 + 100)

    # put limits on the indices
    fx = torch.clamp(fx, 0, 199)
    fy = torch.clamp(fy, 0, 199)

    # cast to long
    fx = fx.long()
    fy = fy.long()

    contact_vector = contact_map[0, 0, fx, fy]
    total_contact_score = torch.sum(contact_vector) / n_steps
    contact_loss = 1.0 - total_contact_score
    l1_loss = torch.nn.L1Loss()(output, target)

    # plt.imshow(contact_map[0, 0, :, :].cpu().numpy())
    # plt.show()

    # print("Output: ", output)
    # print("Contact Vector: ", contact_vector)
    # print("Total Contact Score: ", total_contact_score.item())
    # print("L1 Loss: ", l1_loss.item(), "Contact Loss: ", contact_loss.item())

if __name__ == "__main__":

    home = os.path.expanduser('~')
    model_path = home + '/Downloads/Model_Weights/'
    datasets_path = home + '/Downloads/Planning_Datasets/Basic/'
    train = False
    total_files = 1

    # use arg parser 
    import argparse
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

    # load dataset
    train_dataset, val_dataset = load_dataset(validation_split=0.1)
   
    visualize_raw = args.raw

    if visualize_raw:
        visualize_dataset(train_dataset)    
        exit()

    if train:
        # train and store model
        criterion = footstep_loss
        train_store(train_dataset, val_dataset, batch_size=batch_size, epochs=30, criterion=criterion, model_path=model_path)

    else:
        # load and validate model
        load_validate(train_dataset, batch_size=1, model_path=model_path)


    torch.cuda.empty_cache()



