import torch
import torch.nn.functional as F
import torch.nn as nn
from torch.nn import Module

class FootstepPredictor(Module):
    def __init__(self, input_size, output_size):
        super(FootstepPredictor, self).__init__()

        print("FootstepPredictor (Model) -> Linear Input Size: ", input_size, "Linear Output Size: ", output_size)

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
        
        print_intermediate_output(h1, l1)

        x = torch.cat((h1, l1), dim=1)

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

def print_intermediate_output(h1, l1):
    # print shape, mean, min, max and stddev
    print(
        # "Shapes: ", h1.shape, l1.shape,
        "Mean: ", round(h1.mean().item(), 3), round(l1.mean().item(), 3), 
        "Min: ", round(h1.min().item(), 3), round(l1.min().item(), 3), 
        "Max: ", round(h1.max().item(), 3), round(l1.max().item(), 3), 
        "Stddev: ", round(h1.std().item(), 3), round(l1.std().item(), 3)
    )