import torch
import torch.nn.functional as F

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
    def __init__(self, data, filename):
        
        self.n_steps = 10
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

        self.footstep_plan_sides = get_data(data, 'plan/footstep/side/')
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

            current_plan_positions = self.footstep_plan_positions[i*self.n_steps:(i+1)*self.n_steps, :]
            current_plan_orientations = self.footstep_plan_orientations[i*self.n_steps:(i+1)*self.n_steps, :]

            # check if there are no non-zero norm steps in the plan
            count_footsteps = np.count_nonzero(np.linalg.norm(current_plan_positions, axis=1))

            valid = not(count_footsteps < 6)

            if valid:
                new_height_maps.append(self.height_maps[i])
                new_sensor_positions.append(self.sensor_positions[i, :])
                new_sensor_orientations.append(self.sensor_orientations[i, :])
                new_footstep_plan_positions.append(self.footstep_plan_positions[i*self.n_steps:(i+1)*self.n_steps, :])
                new_footstep_plan_orientations.append(self.footstep_plan_orientations[i*self.n_steps:(i+1)*self.n_steps, :])
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
        footstep_plan_quaternions = self.footstep_plan_orientations[index*self.n_steps:(index+1)*self.n_steps, :]
        footstep_plan_yaws = np.arctan2(2 * (footstep_plan_quaternions[:, 0] * footstep_plan_quaternions[:, 1] + footstep_plan_quaternions[:, 3] * footstep_plan_quaternions[:, 2]),
                    1 - 2 * (footstep_plan_quaternions[:, 0]**2 + footstep_plan_quaternions[:, 3]**2))
        footstep_plan_sides = self.footstep_plan_sides[index*self.n_steps:(index+1)*self.n_steps]
        
        footstep_plan_poses = self.footstep_plan_positions[index*self.n_steps:(index+1)*self.n_steps, :] - sensor_pose
        footstep_plan_poses[:, 2] = footstep_plan_yaws
        # stack such that footstep_plan_poses[:, 3] = footstep_plan_sides
        footstep_plan_poses = np.column_stack((footstep_plan_poses, footstep_plan_sides))
        footstep_plan_poses = np.array(footstep_plan_poses, dtype=np.float32)


        # Inputs
        # --------- Image Inputs (float54)
        height_map_input = torch.Tensor(self.height_maps[index]).unsqueeze(0).to(device)
        
        # --------- Pose Inputs (X, Y, Yaw) with float 64
        start_side = torch.Tensor(start_side).to(device)
        start_pose = torch.Tensor(start_pose).to(device)
        goal_pose = torch.Tensor(goal_pose).to(device)
        # linear_input = torch.cat((start_pose, goal_pose), dim=0) # without start side
        linear_input = torch.cat((start_pose, goal_pose, start_side), dim=0) # with start side

        # Outputs
        # --------- 3D Pose Outputs
        footstep_plan_poses = torch.Tensor(footstep_plan_poses).to(device)
        linear_output = torch.flatten(footstep_plan_poses)

        return height_map_input, linear_input, linear_output
        
    def __len__(self) -> int:
        return len(self.height_maps)


class FootstepPredictor(Module):
    def __init__(self, input_size, output_size):
        super(FootstepPredictor, self).__init__()

        print("FootstepPredictor (Model) -> Linear Input Size: ", input_size, "Linear Output Size: ", output_size)

        # convolutional layers given a 200x200 16-bit grayscale image
        self.conv2d_1 = torch.nn.Conv2d(1, 32, kernel_size=3, stride=1, padding=1)
        self.conv2d_2 = torch.nn.Conv2d(32, 48, kernel_size=3, stride=1, padding=1)
        self.conv2d_3 = torch.nn.Conv2d(48, 64, kernel_size=3, stride=1, padding=1)
        self.conv2d_4 = torch.nn.Conv2d(64, 96, kernel_size=3, stride=1, padding=1)
        self.conv2d_5 = torch.nn.Conv2d(96, 128, kernel_size=3, stride=1, padding=1)
        self.maxpool2d_22 = torch.nn.MaxPool2d(kernel_size=2, stride=2, padding=0)
        self.maxpool2d_44 = torch.nn.MaxPool2d(kernel_size=4, stride=4, padding=0)

        # fully connected layers
        self.fc0 = torch.nn.Linear(input_size, 64)
        self.bn0 = torch.nn.BatchNorm1d(64)
        self.dropout0 = torch.nn.Dropout(0.1)
        self.fc1 = torch.nn.Linear(128 * 6 * 6 + 64, 2048)
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
        h1 = self.conv2d_2(h1)
        h1 = self.maxpool2d_22(h1)
        h1 = self.conv2d_3(h1)
        h1 = self.maxpool2d_22(h1)
        h1 = self.conv2d_4(h1)
        h1 = self.maxpool2d_22(h1)
        h1 = self.conv2d_5(h1)
        h1 = self.maxpool2d_22(h1)
        h1 = torch.flatten(h1, 1)

        l1 = self.fc0(l1)        
        l1 = self.bn0(l1)
        l1 = F.relu(l1)

        # flatten x1 and contactenate with x2
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
    

def train_store(train_dataset, val_dataset, batch_size, epochs, criterion):
    
    train_loader = DataLoader(dataset=train_dataset, batch_size=batch_size, shuffle=True, num_workers=0)
    val_loader = DataLoader(dataset=val_dataset, batch_size=len(val_dataset.indices), shuffle=False, num_workers=0)
    input_size = train_dataset[0][1].shape[0]
    output_size = train_dataset[0][2].shape[0]
    model = FootstepPredictor(input_size, output_size).to(device)

    # define optimizer
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3, weight_decay = 1e-8)

    # train the model
    for epoch in range(epochs):
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

    # save the model
    torch.save(model.state_dict(), 'footstep_predictor.pt')
    torch.onnx.export(model, (x1[0].unsqueeze(0), x2[0].unsqueeze(0)), 'footstep_predictor.onnx', verbose=False)
    writer.close()


def load_validate(val_dataset):
    loader = DataLoader(val_dataset, batch_size=1, shuffle=True)
    
    # Load the model
    input_size = val_dataset[0][1].shape[0]
    output_size = val_dataset[0][2].shape[0]
    model = FootstepPredictor(input_size, output_size)
    model.load_state_dict(torch.load('footstep_predictor.pt'))
    model.eval()
    model.to(device)

    with torch.no_grad():
        for i, (height_map_input, linear_input, target_output) in enumerate(loader):

            height_map_input = height_map_input.to(device)
            linear_input = linear_input.to(device)
            target_output = target_output.to(device)

            predict_output = model(height_map_input, linear_input)

            visualize_output(height_map_input, linear_input, predict_output, i, val_dataset)
            visualize_output(height_map_input, linear_input, target_output, i, val_dataset)

            
def visualize_output(height_map_input, linear_input, final_output, i, val_dataset, n_steps=10):

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
    start_side = linear_input[6]
    plan_poses = output[0:4*n_steps].reshape((n_steps, 4))[:, 0:3]

    # visualize plan
    visualize_plan(height_map, plan_poses, 
                    start_pose, goal_pose)

def load_dataset(validation_split):
    home = os.path.expanduser('~')
    path = home + '/.ihmc/logs/planning-datasets/'
    
    # new_format_files = ['20231018_135001_PerceptionLog.hdf5', 
    #                     '20231018_143108_PerceptionLog.hdf5']
    
    files = \
    [
        "20240212_020130_AStarDataset_Generated.hdf5",      
        "20240212_031849_AStarDataset_Generated_200.hdf5",  
        "20240212_043439_AStarDataset_Generated_400.hdf5",
        "20240212_023954_AStarDataset_Generated_100.hdf5",
        "20240212_035650_AStarDataset_Generated_300.hdf5"
    ]

    
    datasets = []

    for file in files:
        data = h5py.File(path + file, 'r')
        dataset = FootstepDataset(data, file)
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
        train_store(train_dataset, val_dataset, batch_size=10, epochs=30, criterion=criterion)

    else:
        # load and validate model
        load_validate(val_dataset)


    torch.cuda.empty_cache()



