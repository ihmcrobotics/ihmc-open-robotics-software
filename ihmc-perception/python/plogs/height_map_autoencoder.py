import torch
import numpy as np
import matplotlib.pyplot as plt
from torch import nn, optim
from torch.utils.data import DataLoader, Dataset

import torch.nn.functional as F

from torch.utils.data import Dataset, DataLoader
from torch.utils.tensorboard import SummaryWriter

from footstep_dataset_visualizer import visualize_plan
from hdf5_reader import *

from tqdm import tqdm

# Define the encoder architecture
class Encoder(nn.Module):
    def __init__(self):
        super(Encoder, self).__init__()
        self.conv1 = nn.Conv2d(1, 16, kernel_size=3, stride=1, padding=1)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=3, stride=1, padding=1)
        self.conv3 = nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1)
        self.pool = nn.MaxPool2d(kernel_size=2, stride=2)
        self.relu = nn.ReLU()

    def forward(self, x):
        x = self.relu(self.conv1(x))
        x = self.pool(x)
        x = self.relu(self.conv2(x))
        x = self.pool(x)
        x = self.relu(self.conv3(x))
        x = self.pool(x)
        return x

# Define the decoder architecture
class Decoder(nn.Module):
    def __init__(self):
        super(Decoder, self).__init__()
        self.conv1 = nn.ConvTranspose2d(64, 32, kernel_size=4, stride=2, padding=1)
        self.conv2 = nn.ConvTranspose2d(32, 16, kernel_size=4, stride=2, padding=1)
        self.conv3 = nn.ConvTranspose2d(16, 1, kernel_size=3, stride=2, padding=0)
        self.relu = nn.ReLU()

    def forward(self, x):
        x = self.relu(self.conv1(x))
        x = self.relu(self.conv2(x))
        x = self.conv3(x)
        return x

# Define the autoencoder architecture
class Autoencoder(nn.Module):
    def __init__(self):
        super(Autoencoder, self).__init__()
        self.encoder = Encoder()
        self.decoder = Decoder()

    def forward(self, x):
        x = self.encoder(x)
        x = self.decoder(x)
        return x

# Define the dataset class
# create torch tensors on GPU 
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
writer = SummaryWriter("runs/footstep_predictor")

class HeightMapDataset(Dataset):
    def __init__(self, data, filename):
        total_height_maps = len(data['cropped/height/'].keys()) - (len(data['cropped/height/'].keys()) % 10)
        self.height_maps = []
        for i in range(total_height_maps):
            height_map_uint16 = load_depth(data, i, 'cropped/height/')
            height_map_float32 = np.array(height_map_uint16, dtype=np.float32)
            height_map = height_map_float32 / 10000.0
            self.height_maps.append(height_map)
        total_height_maps = len(self.height_maps)
        self.print_size("File Name: " + filename)

    def print_size(self, tag):
        print("Dataset: ------------------------", tag, "-------------------------")
        print(f'Total Height Maps: {len(self.height_maps)}')

    def __getitem__(self, index):
        height_map_input = torch.Tensor(self.height_maps[index]).unsqueeze(0).to(device)        
        return height_map_input, height_map_input
        
    def __len__(self) -> int:
        return len(self.height_maps)


def train_store(train_dataset, val_dataset, batch_size, epochs, criterion):
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False)

    # Train the autoencoder
    model = Autoencoder().to(device)
    num_epochs = epochs

    train_losses = []
    val_losses = []

    optimizer = optim.Adam(model.parameters(), lr=0.001)

    # train the model
    for epoch in range(epochs):
        loop=tqdm(train_loader, bar_format='{l_bar}{bar:30}{r_bar}{bar:-30b}')
        running_loss=0
        for i, (x, y) in enumerate(loop):
            if x.shape[0] > 1:
                x = x.to(device)
                y = y.to(device)

                # Forward pass
                y_pred = model(x)
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

        for x, y in val_loader:
            # send data to gpu
            if x.shape[0] > 1:
                x = x.to(device)
                y = y.to(device)
                model.eval()
                y_pred = model(x)
                loss = criterion(y_pred,y)
                valid_loss = loss.item()
                print("Validiation loss - ",valid_loss,"\n")
                writer.add_scalar('validation loss', valid_loss, epoch * len(train_loader) + i)

        # Store loss
        train_losses.append(average_loss)
        val_losses.append(valid_loss)


    # save the model
    torch.save(model.state_dict(), 'height_map_autoencoder.pt')
    torch.onnx.export(model, (x[0].unsqueeze(0)), 'height_map_autoencoder.onnx', verbose=False)
    writer.close()

    # Plot the training and validation losses
    plt.plot(train_losses, label='Train Loss')
    plt.plot(val_losses, label='Val Loss')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.legend()
    plt.show()



def visualize_dataset(dataset):
    loader = DataLoader(dataset, batch_size=1, shuffle=True)
    for i, (height_map_input, target_output) in enumerate(loader):

        height_map_input = height_map_input.to(device)
        target_output = target_output.to(device)

        height_map = np.array(height_map_input.cpu().numpy() * 10000.0, dtype=np.uint16)
        height_map = height_map.reshape((201, 201))      

        visualize_height_map(height_map)

def visualize_height_map(height_map, target_height_map=None):
    height_map = cv2.convertScaleAbs(height_map, alpha=(255.0/65535.0))
    height_map = np.minimum(height_map * 10, 255)

    height_map_display = height_map.copy()
    height_map_display = cv2.cvtColor(height_map_display, cv2.COLOR_GRAY2RGB)
    height_map_display = cv2.resize(height_map_display, (1000, 1000))

    # if target is not None, stack the target next to the input
    if target_height_map is not None:
        target_height_map = cv2.convertScaleAbs(target_height_map, alpha=(255.0/65535.0))
        target_height_map = np.minimum(target_height_map * 10, 255)
        target_height_map_display = target_height_map.copy()
        target_height_map_display = cv2.cvtColor(target_height_map_display, cv2.COLOR_GRAY2RGB)
        target_height_map_display = cv2.resize(target_height_map_display, (1000, 1000))
        height_map_display = np.hstack((height_map_display, target_height_map_display))

    cv2.namedWindow("Footstep Plan", cv2.WINDOW_NORMAL)
    # cv2.resizeWindow("Footstep Plan", 1000, 1000)
    cv2.imshow("Footstep Plan", height_map_display)
    code = cv2.waitKeyEx(0)

    if code == ord('q'):
        cv2.destroyAllWindows()
        exit()

    return code

def load_dataset(validation_split):
    home = os.path.expanduser('~')
    path = home + '/.ihmc/logs/perception/'
    
    new_format_files = ['20231018_135001_PerceptionLog.hdf5', 
                        '20231018_143108_PerceptionLog.hdf5']
    
    files = \
    [
        # '20231015_183228_PerceptionLog.hdf5', 
        # '20231015_234600_PerceptionLog.hdf5', 
        # '20231016_025456_PerceptionLog.hdf5',
        '20231023_131517_PerceptionLog.hdf5',
        '20231023_160823_PerceptionLog.hdf5',
        # '20231028_171524_PerceptionLog.hdf5',
    ]
    
    datasets = []

    for file in files:
        data = h5py.File(path + file, 'r')
        dataset = HeightMapDataset(data, file)
        datasets.append(dataset)

    dataset = torch.utils.data.ConcatDataset(datasets)
    val_size = int(validation_split * len(dataset))
    train_size = len(dataset) - val_size
    train_dataset, val_dataset = torch.utils.data.random_split(dataset, [train_size, val_size])

    return train_dataset, val_dataset

def load_validate(val_dataset):
    loader = DataLoader(val_dataset, batch_size=1, shuffle=False)
    
    # Load the model
    # input_size = val_dataset[0][1].shape[0]
    # output_size = val_dataset[0][2].shape[0]
    model = Autoencoder()
    model.load_state_dict(torch.load('height_map_autoencoder.pt'))
    model.eval()
    model.to(device)

    with torch.no_grad():
        for i, (height_map_input, target_output) in enumerate(loader):

            height_map_input = height_map_input.to(device)
            target_output = target_output.to(device)

            predict_output = model(height_map_input)
            predict_output = np.array(predict_output.cpu().numpy() * 10000.0, dtype=np.uint16)
            predict_output = predict_output.reshape((201, 201))      

            target_output = np.array(target_output.cpu().numpy() * 10000.0, dtype=np.uint16)
            target_output = target_output.reshape((201, 201))

            visualize_height_map(predict_output, target_output)

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
        train_store(train_dataset, val_dataset, batch_size=10, epochs=200, criterion=criterion)

    else:
        # load and validate model
        load_validate(val_dataset)

    torch.cuda.empty_cache()



