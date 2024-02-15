import torch
import gc
torch.cuda.empty_cache()

import torch.nn.functional as F
import torch.nn as nn
from torch.nn import Module, Sequential, Linear, ReLU, Dropout, BatchNorm1d
from torch.utils.data import Dataset, DataLoader
from torch.utils.tensorboard import SummaryWriter

from footstep_dataset_loader import visualize_plan
from hdf5_reader import *

from tqdm import tqdm

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

        # Inputs
        # --------- Image Inputs (float54)
        height_map_input = torch.Tensor(self.height_maps[index]).unsqueeze(0).to(device)
        
        # --------- Pose Inputs (X, Y, Yaw) with float 64
        start_pose = torch.Tensor(start_pose).to(device)
        goal_pose = torch.Tensor(goal_pose).to(device)
        linear_input = torch.cat((start_pose, goal_pose), dim=0) 

        # Outputs
        # --------- 3D Pose Outputs
        footstep_plan_poses = torch.Tensor(footstep_plan_poses).to(device)
        linear_output = torch.flatten(footstep_plan_poses)

        return height_map_input, linear_input, linear_output
        
    def __len__(self) -> int:
        return len(self.height_maps)


class ResidualBlock(nn.Module):
    def __init__(self, in_channels, out_channels, stride=1):
        super(ResidualBlock, self).__init__()
        self.conv1 = nn.Conv2d(in_channels, out_channels, kernel_size=3, stride=stride, padding=1, bias=False)
        self.bn1 = nn.BatchNorm2d(out_channels)
        self.conv2 = nn.Conv2d(out_channels, out_channels, kernel_size=3, stride=1, padding=1, bias=False)
        self.bn2 = nn.BatchNorm2d(out_channels)
        self.relu = nn.ReLU(inplace=True)
        self.downsample = None
        if stride != 1 or in_channels != out_channels:
            self.downsample = nn.Sequential(
                nn.Conv2d(in_channels, out_channels, kernel_size=1, stride=stride, bias=False),
                nn.BatchNorm2d(out_channels)
            )

    def forward(self, x):
        residual = x
        out = self.conv1(x)
        out = self.bn1(out)
        out = self.relu(out)
        out = self.conv2(out)
        out = self.bn2(out)
        if self.downsample is not None:
            residual = self.downsample(x)
        out += residual
        out = self.relu(out)
        return out

class FootstepPredictor(Module):
    def __init__(self, input_size, output_size):
        super(FootstepPredictor, self).__init__()

        print("FootstepPredictor (Model) -> Linear Input Size: ", input_size, "Linear Output Size: ", output_size)

        # convolutional layers given a 200x200 16-bit grayscale image
        self.conv2d_1 = torch.nn.Conv2d(1, 32, kernel_size=3, stride=1, padding=1)
        self.conv2d_2 = ResidualBlock(32, 48)
        self.conv2d_3 = ResidualBlock(48, 64)
        self.conv2d_4 = ResidualBlock(64, 96)
        self.maxpool2d_22 = torch.nn.MaxPool2d(kernel_size=2, stride=2, padding=0)
        self.maxpool2d_44 = torch.nn.MaxPool2d(kernel_size=4, stride=4, padding=0)

        # fully connected layers
        self.fc0 = torch.nn.Linear(input_size, 4096)
        self.bn0 = torch.nn.BatchNorm1d(4096)
        self.dropout0 = torch.nn.Dropout(0.1)
        self.fc1 = torch.nn.Linear(96 * 12 * 12 + 4096, 2048)
        self.bn1 = torch.nn.BatchNorm1d(2048)
        self.dropout1 = torch.nn.Dropout(0.1)
        self.fc2 = torch.nn.Linear(2048, 1024)
        self.bn2 = torch.nn.BatchNorm1d(1024)
        self.fc3 = torch.nn.Linear(1024, 512)
        self.bn3 = torch.nn.BatchNorm1d(512)
        self.fc4 = torch.nn.Linear(512, 256)
        self.bn4 = torch.nn.BatchNorm1d(256)
        self.fc5 = torch.nn.Linear(256, 128)
        self.bn5 = torch.nn.BatchNorm1d(128)
        self.fc6 = torch.nn.Linear(128, output_size)

    def forward(self, h1, l1):

        print("\n\nForward Pass Input Shape: ", h1.shape, l1.shape, end=" ")

        # x1 is image 200x200 16-bit grayscale and x2 is the 3D pose
        h1 = self.conv2d_1(h1)
        h1 = self.maxpool2d_22(h1)
        residual_1 = h1
        h1 = self.conv2d_2(h1)
        h1 = self.maxpool2d_22(h1)
        residual_2 = h1
        h1 = self.conv2d_3(h1)
        h1 = self.maxpool2d_22(h1)
        residual_3 = h1
        h1 = self.conv2d_4(h1)
        h1 = self.maxpool2d_22(h1)

        h1 = torch.flatten(h1, 1)

        l1 = self.fc0(l1)        
        l1 = self.bn0(l1)
        l1 = F.relu(l1)
        
        print("Shapes: ", h1.shape, l1.shape)

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

        print("Forward Pass Output Shape: ", x.shape)

        return x

def train_store(train_dataset, val_dataset, batch_size, epochs, criterion, model_path):
    
    train_loader = DataLoader(dataset=train_dataset, batch_size=batch_size, shuffle=True, num_workers=0)
    val_loader = DataLoader(dataset=val_dataset, batch_size=batch_size, shuffle=False, num_workers=0)
    input_size = train_dataset[0][1].shape[0]
    output_size = train_dataset[0][2].shape[0]
    model = FootstepPredictor(input_size, output_size).to(device)

    # define optimizer
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3, weight_decay = 1e-8)

    # train the model
    for epoch in range(epochs):
        gc.collect()
        torch.cuda.empty_cache()

        loop=tqdm(train_loader, bar_format='{l_bar}{bar:30}{r_bar}{bar:-30b}')
        running_loss=0
        for i, (x1, x2, y) in enumerate(loop):
            if x1.shape[0] > 1:
                x1 = x1.to(device)
                x2 = x2.to(device)
                y = y.to(device)

                # Forward pass
                y_pred = model(x1, x2)
                loss = criterion(y_pred, y)

                # Backward pass and optimize
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

                # calculate running loss
                running_loss+=loss.item()*y.size()[0]
                average_loss = running_loss/(y.size(0)*(i+1))
                loop.set_description(f'Epoch [{epoch+1}/{epochs}]')
                loop.set_postfix(loss=average_loss)
                writer.add_scalar('training loss', average_loss, epoch * len(train_loader) + i)

        for x1, x2, y in val_loader:
            # send data to gpu
            if x1.shape[0] > 1:
                x1 = x1.to(device)
                x2 = x2.to(device)
                y = y.to(device)
                model.eval()
                y_pred = model(x1, x2)
                loss = criterion(y_pred,y)
                valid_loss = loss.item()
                print("Validiation loss - ",valid_loss,"\n")
                writer.add_scalar('validation loss', valid_loss, epoch * len(train_loader) + i)

        del x1, x2, y, y_pred, loss
        gc.collect()
        torch.cuda.empty_cache()

    # save the model
    torch.save(model.state_dict(), model_path + 'footstep_predictor.pt')
    torch.onnx.export(model, (x1[0].unsqueeze(0), x2[0].unsqueeze(0)), model_path + 'footstep_predictor.onnx', verbose=False)
    writer.close()


def load_validate(val_dataset, batch_size, model_path):
    loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=True)
    
    # Load the model
    input_size = val_dataset[0][1].shape[0]
    output_size = val_dataset[0][2].shape[0]
    model = FootstepPredictor(input_size, output_size)
    model.load_state_dict(torch.load(model_path + 'footstep_predictor.pt'))
    model.eval()
    model.to(device)

    with torch.no_grad():
        for i, (height_map_input, linear_input, target_output) in enumerate(loader):

            height_map_input = height_map_input.to(device)
            linear_input = linear_input.to(device)
            target_output = target_output.to(device)

            predict_output = model(height_map_input, linear_input)

            # get only the first 4 steps
            predict_output = predict_output[0:3*n_steps, :].reshape((n_steps, 3))

            visualize_output(height_map_input, linear_input, predict_output, i, val_dataset, label="Prediction")
            visualize_output(height_map_input, linear_input, target_output, i, val_dataset, label="Target")

            
def visualize_output(height_map_input, linear_input, final_output, i, val_dataset, n_steps=4, label="Footstep Plan"):

    output = final_output.cpu().numpy()
    output = output.squeeze()
    print("Predict Output", output.shape)

    linear_input = linear_input.cpu().numpy()
    linear_input = linear_input.squeeze()
    print("Linear Input", linear_input.shape)

    print(f'Dataset: {i}/{len(val_dataset)}, Height Map Size: {height_map_input.shape}, Linear Input Size: {linear_input.shape}, Linear Output Size: {output.shape}')

    # convert height map to numpy array 16-bit grayscale
    height_map = np.array(height_map_input.cpu().numpy() * 10000.0, dtype=np.uint16)

    # reshape for opencv
    height_map = height_map.reshape((201, 201))        

    start_pose = linear_input[0:3]
    goal_pose = linear_input[3:6]
    plan_poses = output[0:3*n_steps].reshape((n_steps, 3))[:, 0:3]

    # visualize plan
    visualize_plan(height_map, plan_poses, 
                    start_pose, goal_pose, label=label)

def load_dataset(validation_split):
    home = os.path.expanduser('~')
    path = home + '/Downloads/Planning_Datasets/'
    
    # input_files = ['20231018_135001_PerceptionLog.hdf5', 
    #                     '20231018_143108_PerceptionLog.hdf5']
    
    files = sorted(os.listdir(path))
    files = [file for file in files if ".hdf5" in file]

    labels = [
                # 'MCFP', 
                'AStar'
    ]

    # filter by label
    files = [file for file in files if any(label in file for label in labels)]
    

    # files = files[:6]

    
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

def visualize_dataset(dataset):
    loader = DataLoader(dataset, batch_size=1, shuffle=True)
    for i, (height_map_input, linear_input, target_output) in enumerate(loader):

        height_map_input = height_map_input.to(device)
        linear_input = linear_input.to(device)
        target_output = target_output.to(device)


        visualize_output(height_map_input, linear_input, target_output, i, val_dataset)

if __name__ == "__main__":

    home = os.path.expanduser('~')
    model_path = home + '/Downloads/Model_Weights/'
    datasets_path = home + '/Downloads/Planning_Datasets/'

    n_steps = 4

    # load dataset
    train_dataset, val_dataset = load_dataset(validation_split=0.05)
   
    train = False
    visualize_raw = False

    if visualize_raw:
        visualize_dataset(train_dataset)    
        exit()

    if train:
        # train and store model
        criterion=torch.nn.L1Loss()
        train_store(train_dataset, val_dataset, batch_size=2, epochs=30, criterion=criterion, model_path=model_path)

    else:
        # load and validate model
        load_validate(val_dataset, batch_size=1, model_path=model_path)


    torch.cuda.empty_cache()



