import torch
import torch.nn.functional as F

from torch.nn import Module, Sequential, Linear, ReLU, Dropout, BatchNorm1d

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

if __name__ == "__main__":
    
    # train and store model
    train_store()

    # load and validate model
    load_validate()