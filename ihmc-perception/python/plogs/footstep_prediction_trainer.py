import torch
import torch.nn.functional as F

from torch.nn import Module, Sequential, Linear, ReLU, Dropout, BatchNorm1d
from torch.utils.data import Dataset, DataLoader

from footstep_dataset_loader import visualize_plan
from hdf5_reader import *

from tqdm import tqdm

# create torch tensors on GPU 
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

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

        total_height_maps = len(self.height_maps)

        self.print_size("Before Removal")

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

            current_plan_positions = self.footstep_plan_positions[i*10:(i+1)*10, :]
            current_plan_orientations = self.footstep_plan_orientations[i*10:(i+1)*10, :]

            # check if there are no non-zero norm steps in the plan
            count_footsteps = np.count_nonzero(np.linalg.norm(current_plan_positions, axis=1))

            valid = not(count_footsteps < 6)

            if valid:
                new_height_maps.append(self.height_maps[i])
                new_sensor_positions.append(self.sensor_positions[i, :])
                new_sensor_orientations.append(self.sensor_orientations[i, :])
                new_footstep_plan_positions.append(self.footstep_plan_positions[i*10:(i+1)*10, :])
                new_footstep_plan_orientations.append(self.footstep_plan_orientations[i*10:(i+1)*10, :])
                new_start_positions.append(self.start_positions[i, :])
                new_start_orientations.append(self.start_orientations[i, :])
                new_goal_positions.append(self.goal_positions[i, :])
                new_goal_orientations.append(self.goal_orientations[i, :])

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
        print("Dataset Tag: ", tag, "-------------------------------------------------")
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

        # Inputs
        # --------- Image Inputs
        height_map_input = torch.from_numpy(self.height_maps[index]).to(device).unsqueeze(0)
        
        # --------- 3D Pose Inputs
        sensor_position = torch.from_numpy(self.sensor_positions[index, :]).to(device)
        # sensor_orientation = torch.from_numpy(self.sensor_orientations[index, :]).to(device)
        start_position = torch.from_numpy(self.start_positions[index, :2]).to(device) - sensor_position[:2]
        start_orientation = torch.from_numpy(self.start_orientations[index, :] ).to(device)
        goal_position = torch.from_numpy(self.goal_positions[index, :2]).to(device) - sensor_position[:2]
        goal_orientation = torch.from_numpy(self.goal_orientations[index, :] ).to(device)
        linear_input = torch.cat((start_position, goal_position), dim=0)

        # Outputs
        # --------- 3D Pose Outputs
        current_plan_positions = torch.from_numpy(self.footstep_plan_positions[index*10:(index+1)*10, :]).to(device) - sensor_position
        # current_plan_orientations = torch.from_numpy(self.footstep_plan_orientations[index*10:(index+1)*10, :]).to(device)

        linear_output = torch.flatten(current_plan_positions[:,:2])

        return height_map_input, linear_input, linear_output
        
    def __len__(self) -> int:
        return len(self.height_maps)





class FootstepPredictor(Module):
    def __init__(self, input_size, output_size):
        super(FootstepPredictor, self).__init__()

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
        self.dropout0 = torch.nn.Dropout(0.5)
        self.fc1 = torch.nn.Linear(128 * 6 * 6 + 64, 2048)
        self.bn1 = torch.nn.BatchNorm1d(2048)
        self.dropout1 = torch.nn.Dropout(0.5)
        self.fc2 = torch.nn.Linear(2048, 1024)
        self.bn2 = torch.nn.BatchNorm1d(1024)
        self.fc3 = torch.nn.Linear(1024, 512)
        self.bn3 = torch.nn.BatchNorm1d(512)
        self.fc4 = torch.nn.Linear(512, 256)
        self.bn4 = torch.nn.BatchNorm1d(256)
        self.fc5 = torch.nn.Linear(256, 128)
        self.bn5 = torch.nn.BatchNorm1d(128)
        self.fc6 = torch.nn.Linear(128, output_size)


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

        x2 = self.fc0(x2)
        x2 = self.bn0(x2)
        x2 = F.relu(x2)

        # flatten x1 and contactenate with x2
        x = torch.cat((x1, x2), dim=1)

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

        for x1, x2, y in val_loader:
            # send data to gpu
            x1 = x1.to(device)
            x2 = x2.to(device)
            y = y.to(device)
            model.eval()
            y_pred = model(x1, x2)
            loss = criterion(y_pred,y)
            valid_loss = loss.item()
            print("Validiation loss - ",valid_loss,"\n")

    # save the model
    torch.save(model.state_dict(), 'footstep_predictor.pt')
    torch.onnx.export(model, (x1[0].unsqueeze(0), x2[0].unsqueeze(0)), 'footstep_predictor.onnx', verbose=False)


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
            predict_output = predict_output.cpu().numpy()
            predict_output = predict_output.squeeze()
            print("Predict Output", predict_output.shape)

            target_output = target_output.cpu().numpy()
            target_output = target_output.squeeze()
            print("Target Output", target_output.shape)

            linear_input = linear_input.cpu().numpy()
            linear_input = linear_input.squeeze()
            print("Linear Input", linear_input.shape)

            print(f'Dataset: {i}/{len(val_dataset)}, Height Map Size: {height_map_input.shape}, Linear Input Size: {linear_input.shape}, Linear Output Size: {target_output.shape}')

            # convert height map to numpy array 16-bit grayscale
            height_map = np.array(height_map_input.cpu().numpy() * 10000.0, dtype=np.uint16)

            # reshape for opencv
            height_map = height_map.reshape((201, 201))        

            print("Height Map Shape: ", height_map.shape)

            # split linear input into start and goal positions x and y, set z and orientations to zero
            start_position = linear_input[0:2]
            start_position = np.hstack((start_position, np.zeros((1))))
            start_orientation = np.zeros((4))
            goal_position = linear_input[2:4]
            goal_position = np.hstack((goal_position, np.zeros((1))))
            goal_orientation = np.zeros((4))

            # split linear output into current plan positions (x, y) and zero z to positions, set orientations to zero
            current_plan_positions = target_output[0:20].reshape((10, 2))
            current_plan_positions = np.hstack((current_plan_positions, np.zeros((10, 1))))
            current_plan_orientations = np.zeros((10, 4))            

            # split predict output into current plan positions similarly
            predict_plan_positions = predict_output[0:20].reshape((10, 2))
            predict_plan_positions = np.hstack((predict_plan_positions, np.zeros((10, 1))))
            predict_plan_orientations = np.zeros((10, 4))

            print("Target Shape; ", target_output.shape, "Predict Shape: ", predict_output.shape)

            # visualize plan
            visualize_plan(height_map, predict_plan_positions, predict_plan_orientations, 
                           start_position, start_orientation, goal_position, goal_orientation,
                            i, len(val_dataset))


def load_dataset(validation_split):
    home = os.path.expanduser('~')
    path = home + '/.ihmc/logs/perception/'
    # new_format_files = ['20231018_135001_PerceptionLog.hdf5']#, '20231018_143108_PerceptionLog.hdf5']
    files = ['20231015_183228_PerceptionLog.hdf5', '20231015_234600_PerceptionLog.hdf5', '20231016_025456_PerceptionLog.hdf5']
    datasets = []

    for file in files:
        data = h5py.File(path + file, 'r')
        dataset = FootstepDataset(data)
        datasets.append(dataset)

    dataset = torch.utils.data.ConcatDataset(datasets)
    val_size = int(validation_split * len(dataset))
    train_size = len(dataset) - val_size
    train_dataset, val_dataset = torch.utils.data.random_split(dataset, [train_size, val_size])

    return train_dataset, val_dataset

if __name__ == "__main__":

    # load dataset
    train_dataset, val_dataset = load_dataset(validation_split=0.05)
   
    train = False

    if train:
        # train and store model
        criterion=torch.nn.L1Loss()
        train_store(train_dataset, val_dataset, batch_size=32, epochs=20, criterion=criterion)

    else:
        # load and validate model
        load_validate(val_dataset)

    torch.cuda.empty_cache()



