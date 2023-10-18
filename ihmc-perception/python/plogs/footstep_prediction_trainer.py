import torch
import torch.nn.functional as F

from torch.nn import Module, Sequential, Linear, ReLU, Dropout, BatchNorm1d
from torch.utils.data import Dataset, DataLoader

from footstep_dataset_loader import visualize_plan
from hdf5_reader import *

class FootstepDataset(Dataset):
    def __init__(self, data):
        
        total_height_maps = len(data['cropped/height/'].keys()) - len(data['cropped/height/'].keys()) % 10

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

        self.start_positions = get_data(data, 'start/footstep/position/')
        self.start_orientations = get_data(data, 'start/footstep/orientation/')

        self.goal_positions = get_data(data, 'goal/footstep/position/')
        self.goal_orientations = get_data(data, 'goal/footstep/orientation/')

        print(f'Total Height Maps: {len(self.height_maps)}')
        print(f'Total Sensor Positions: {len(self.sensor_positions)}')
        print(f'Total Sensor Orientations: {len(self.sensor_orientations)}')
        print(f'Total Footstep Plan Positions: {len(self.footstep_plan_positions)}')
        print(f'Total Footstep Plan Orientations: {len(self.footstep_plan_orientations)}')
        print(f'Total Start Positions: {len(self.start_positions)}')
        print(f'Total Start Orientations: {len(self.start_orientations)}')
        print(f'Total Goal Positions: {len(self.goal_positions)}')
        print(f'Total Goal Orientations: {len(self.goal_orientations)}')


    def __getitem__(self, index):

        # Inputs
        # --------- Image Inputs
        height_map_input = self.height_maps[index]
        
        # --------- 3D Pose Inputs
        sensor_position = self.sensor_positions[index, :]
        sensor_orientation = self.sensor_orientations[index, :]
        
        start_position = self.start_positions[index, :] - sensor_position
        start_orientation = self.start_orientations[index, :] 
        goal_position = self.goal_positions[index, :] - sensor_position
        goal_orientation = self.goal_orientations[index, :] 

        # create torch tensors on GPU 
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        height_map_input = torch.from_numpy(height_map_input).to(device)
        start_position = torch.from_numpy(start_position).to(device)
        start_orientation = torch.from_numpy(start_orientation).to(device)
        goal_position = torch.from_numpy(goal_position).to(device)
        goal_orientation = torch.from_numpy(goal_orientation).to(device)

        linear_input = torch.cat((start_position, start_orientation, goal_position, goal_orientation), dim=0)

        # Outputs
        # --------- 3D Pose Outputs
        current_plan_positions = self.footstep_plan_positions[index*10:(index+1)*10, :]
        current_plan_orientations = self.footstep_plan_orientations[index*10:(index+1)*10, :]

        # create torch tensors on GPU
        current_plan_positions = torch.from_numpy(current_plan_positions).to(device)
        current_plan_orientations = torch.from_numpy(current_plan_orientations).to(device)

        linear_output = torch.cat((torch.flatten(current_plan_positions), torch.flatten(current_plan_orientations)), dim=0)

        return height_map_input, linear_input, linear_output
        
    def __len__(self) -> int:
        return len(self.height_maps)

class FootstepPredictor(Module):
    def __init__(self, output_size):
        super(FootstepPredictor, self).__init__()
        
        # convolutional layers given a 200x200 16-bit grayscale image
        self.conv2d_1 = torch.nn.Conv2d(1, 32, kernel_size=3, stride=1, padding=1)
        self.conv2d_2 = torch.nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1)
        self.conv2d_3 = torch.nn.Conv2d(64, 128, kernel_size=3, stride=1, padding=1)
        self.conv2d_4 = torch.nn.Conv2d(128, 64, kernel_size=3, stride=1, padding=1)
        self.conv2d_5 = torch.nn.Conv2d(64, 32, kernel_size=3, stride=1, padding=1)
        self.maxpool2d_22 = torch.nn.MaxPool2d(kernel_size=2, stride=2, padding=0)
        self.maxpool2d_44 = torch.nn.MaxPool2d(kernel_size=4, stride=4, padding=0)

        # fully connected layers
        self.fc1 = torch.nn.Linear(32 * 6 * 6 + 200, 1024)
        self.fc2 = torch.nn.Linear(1024, 512)
        self.fc3 = torch.nn.Linear(512, output_size)


    def forward(self, x1, x2):

        # x1 is image 200x200 16-bit grayscale and x2 is the 3D pose
        x1 = self.conv2d_1(x1)
        x1 = self.maxpool2d_22(x1)
        x1 = self.conv2d_2(x1)
        x1 = self.maxpool2d_22(x1)
        x1 = self.conv2d_3(x1)
        x1 = self.maxpool2d_22(x1)
        x1 = self.conv2d_4(x1)
        x1 = self.maxpool2d_22(x1)
        x1 = self.conv2d_5(x1)
        x1 = self.maxpool2d_22(x1)
        x1 = torch.flatten(x1, 1)

        # flatten x1 and contactenate with x2
        x = torch.cat((x1, x2), dim=1)

        # fully connected layers
        x = self.fc1(x)
        x = F.relu(x)
        x = self.fc2(x)
        x = F.relu(x)
        x = self.fc3(x)

        return x
    
def train_store():
    # test
    output_size = 256
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = FootstepPredictor(output_size).to(device)
    print(model)

    epochs = 10

    x1 = torch.randn(100, 1, 200, 200).to(device)
    x2 = torch.randn(100, 200).to(device)
    y = torch.randn(100, output_size).to(device)

    # define loss function
    criterion = torch.nn.CrossEntropyLoss()

    # define optimizer
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)

    # train the model
    for epoch in range(epochs):

        # forward pass
        y_pred = model(x1, x2)


        # compute loss
        loss = criterion(y_pred, y)

        # zero gradients
        optimizer.zero_grad()

        # backward pass
        loss.backward()

        # update weights
        optimizer.step()

        # print loss
        print('epoch: ', epoch, ' loss: ', loss.item())

    # save model
    torch.save(model.state_dict(), 'footstep_predictor.pt')

    # export model to onnx
    export_onnx(model, x1, x2, y)

def export_onnx(model, x1, x2, y):
    # export model to onnx for a single image not full batch
    torch.onnx.export(model, (x1[0].unsqueeze(0), x2[0].unsqueeze(0)), 'footstep_predictor.onnx', verbose=True)

def load_validate():
    # load model
    model = FootstepPredictor(256)
    model.load_state_dict(torch.load('footstep_predictor.pt'))

    # validate model
    x1 = torch.randn(1, 1, 200, 200)
    x2 = torch.randn(1, 200)
    y = torch.randn(1, 256)

    y_pred = model(x1, x2)
    print(y_pred)

def load_dataset():
    home = os.path.expanduser('~')
    path = home + '/.ihmc/logs/perception/'

    data = h5py.File(path + '20231015_183228_PerceptionLog.hdf5', 'r')
    dataset = FootstepDataset(data)

    return dataset

if __name__ == "__main__":

    # list of good files
    # 20231015_183228_PerceptionLog.hdf5
    # 20231015_234600_PerceptionLog.hdf5
    # 20231016_025456_PerceptionLog.hdf5

    # load dataset
    dataset = load_dataset()

    for i in range(len(dataset)):

        height_map_input, linear_input, linear_output = dataset[i]

        print(f'Dataset: {i}/{len(dataset)}, Height Map Size: {height_map_input.shape}, Linear Input Size: {linear_input.shape}, Linear Output Size: {linear_output.shape}')

        # convert height map to numpy array 16-bit grayscale
        height_map = np.array(height_map_input.cpu().numpy() * 10000.0, dtype=np.uint16)

        # convert linear input to numpy array
        linear_input = linear_input.cpu().numpy()

        # split linear input into start and goal positions and orientations
        start_position = linear_input[0:3]
        start_orientation = linear_input[3:7]
        goal_position = linear_input[7:10]
        goal_orientation = linear_input[10:14]

        # convert linear output to numpy array
        linear_output = linear_output.cpu().numpy()

        # split linear output into current plan positions and orientations
        current_plan_positions = linear_output[0:30].reshape((10, 3))
        current_plan_orientations = linear_output[30:70].reshape((10, 4))

        # visualize plan
        visualize_plan(height_map, current_plan_positions, current_plan_orientations, start_position,
                       start_orientation, goal_position, goal_orientation,
                       i, len(dataset))

    # train and store model
    # train_store(dataset)

    # # load and validate model
    # load_validate(dataset)