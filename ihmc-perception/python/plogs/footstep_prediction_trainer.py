import torch
import gc
torch.cuda.empty_cache()

from torch.utils.data import Dataset, DataLoader
from torch.utils.tensorboard import SummaryWriter

from tqdm import tqdm

import os.path
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from plotting.height_map_tools import *
from footstep_dataset_visualizer import visualize_plan
from hdf5_reader import *

# create torch tensors on GPU 
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
writer = SummaryWriter("runs/footstep_predictor")

from footstep_prediction_model_flat import *
from footstep_dataset_loader import *


def train_store(train_dataset, val_dataset, batch_size, epochs, criterion, model_path, warm_start=False,):
    
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

    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)

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
    input_size = val_dataset[0][1].shape[0]
    output_size = val_dataset[0][2].shape[0]
    model = FootstepPredictor(input_size, output_size)
    model_files = sorted([name for name in os.listdir(model_path) if name.endswith('.pt')])

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
            predict_output = predict_output[0:3*n_steps, :].reshape((n_steps, 3))
            target_output = target_output[0:3*n_steps].reshape((n_steps, 3))

            visualize_output(height_map_input, linear_input, predict_output, contact_map, terrain_cost, label="Prediction")
            visualize_output(height_map_input, linear_input, target_output, contact_map, terrain_cost, label="Target")

            
def visualize_output(height_map_input, linear_input, final_output, contact_map_uint8, terrain_cost, n_steps=4, label="Footstep Plan"):

    output = final_output.cpu().numpy()
    output = output.squeeze()

    linear_input = linear_input.cpu().numpy()
    linear_input = linear_input.squeeze()

    height_map_uint16 = np.array(height_map_input.cpu().numpy() * 10000.0, dtype=np.uint16)

    if flat:
        height_map_uint16 = height_map_uint16[:, 0, :, :]

    height_map_uint16 = height_map_uint16.reshape((201, 201))        

    terrain_cost = np.array(terrain_cost.cpu().numpy(), dtype=np.uint8)
    terrain_cost = terrain_cost.reshape((201, 201))

    contact_map_uint8 = np.array(contact_map_uint8.cpu().numpy(), dtype=np.uint8)
    contact_map_uint8 = contact_map_uint8.reshape((201, 201))

    start_pose = linear_input[0:3]
    goal_pose = linear_input[3:6]
    plan_poses = output[0:3*n_steps].reshape((n_steps, 3))[:, 0:3]

    visualize_plan(height_map_uint16, contact_map_uint8, terrain_cost, plan_poses, 
                    start_pose, goal_pose, label=label)

def load_dataset(validation_split, datasets_path, count, filter, flat):
    
    files = os.listdir(datasets_path)
    files.sort(reverse=True)
    files = [file for file in files if ".hdf5" in file]

    labels = [
                'MCFP', 
                'AStar',
    ]

    print("---------------++++++++++++++++++++ Files to Load: ", count, "/", len(files), " +++++++++++++++++++-------------------")
    for file in files:
        print("File Found: ", file)

    # filter by label
    files = [file for file in files if any(label in file for label in labels)]
    files = files[:count]
    
    datasets = []

    for file in files:

        print("Loading File: ", file)

        data = h5py.File(datasets_path + file, 'r')
        dataset = FootstepDataset(data, file, n_steps=n_steps, flat=flat)
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

    contact_loss = compute_contact_loss(output, target, contact_map)
    return torch.nn.L1Loss()(output, target) + contact_loss
        
# def footstep_loss(output, target, contact_map):

#     # Reshape the output tensor to extract footstep positions [Output Shape: (8, 12)]

#     print("Output Shape: ", output.shape)

#     output_reshaped = output.view(-1, 4, 3)
#     # output_reshaped = output.view(-1, 4, 3)

#     print("Output Reshaped: ", output_reshaped.shape)

#     # Extract x and y coordinates of predicted footstep positions
#     fx = output_reshaped[:, :, 0]
#     fy = output_reshaped[:, :, 1]

#     print("FX: ", fx.shape)
#     print("FY: ", fy.shape)

#     # Convert footstep positions to pixel indices and clamp within valid range
#     fx = (fx * 50 + 100).clamp(0, 199).long()
#     fy = (fy * 50 + 100).clamp(0, 199).long()

#     # Compute contact scores from the contact map at predicted footstep positions

#     # query the contact map of shape (batch_size,1,x_coord,y_coord) with fx of shape (batch_size,x_coord) and fy of shape (batch_size,y_coord) 
#     contact_scores = contact_map[:, 0, fx, fy]


#     # contact_scores = contact_scores.sum(dim=1) / 4.0

#     print("Contact Scores: ", contact_scores.shape)

#     # Calculate contact loss as the difference between predicted and actual contact scores
#     contact_loss = (1.0 - contact_scores) * 100.0

#     print("Contact Loss: ", contact_loss.shape)   

#     # Compute L1 loss between predicted and target footstep positions
#     l1_loss = torch.nn.L1Loss()(output, target)

#     # Combine L1 loss and contact map loss
#     total_loss = l1_loss + contact_loss

#     print("Shapes: ", contact_loss.shape, l1_loss.shape, total_loss.shape)
    
#     return total_loss


def compute_contact_loss_vectorized(output, target, contact_map):
    print("Output Shape: ", output.shape, "Target Shape: ", target.shape, "Contact Map Shape: ", contact_map.shape)

    output_reshaped = torch.reshape(output, (-1, 4, 3))
    fx = output_reshaped[:, :, 0]
    fy = output_reshaped[:, :, 1]
    contact = contact_map[:, 0, fx.long(), fy.long()]  # Fix: Use contact_map provided in batch
    contact_loss = -contact

    print("FX: ", fx)
    print("FY: ", fy)

    contact_vector = contact_map[fx, fy]
    print("Unique: ", torch.unique(contact_vector))

    total_contact_score = torch.sum(contact_vector) / n_steps
    contact_loss = 1.0 - total_contact_score

    print("Contact Vector: ", contact_vector)
    print("Total Contact Score: ", total_contact_score.item())

    l1_loss = torch.nn.L1Loss()(output, target)

    return (l1_loss + contact_loss) / 2.0

def compute_contact_loss(output, target, contact_map):
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
        
        total_contact_score = per_image_score / n_steps
        contact_loss = (1.0 - total_contact_score) * 100.0

        sum_loss.append((contact_loss))
    
    return sum(sum_loss) / len (sum_loss)

if __name__ == "__main__":

    home = os.path.expanduser('~')
    train = False
    total_files = 1

    # use arg parser 
    import argparse
    parser = argparse.ArgumentParser(description='Footstep Prediction Trainer')
    parser.add_argument("--model_path", help="path to store model", type=str, default="Downloads/Model_Weights")
    parser.add_argument("--data_path", help="path to search for files", type=str, default="Downloads/Planning_Datasets")
    parser.add_argument('--train', action='store_true', help='Train the model', default=False)
    parser.add_argument('--files', type=int, help='Total Files to Load', default=-1)
    parser.add_argument('--raw', action='store_true', help='Raw Visualization')
    parser.add_argument("--lr", type=float, help="Learning Rate", default=1e-3)
    parser.add_argument("--flat", action='store_true', help='Flat Model', default=False)
    parser.add_argument("--batch_size", type=int, help="Batch Size", default=8)
    parser.add_argument("--epochs", type=int, help="Number of Epochs", default=30)
    parser.add_argument("--n_steps", type=int, help="Number of Steps", default=4)
    parser.add_argument("--split", type=float, help="Validation Split", default=0.1)
    parser.add_argument("--filter", type=str, help="Filter Files", default=".hdf5")

    args = parser.parse_args()

    total_files = args.files
    model_path = home + '/' + args.model_path + '/'
    datasets_path = home + '/' + args.data_path + '/'
    validation_split = args.split
    batch_size = args.batch_size
    n_steps = args.n_steps
    epochs = args.epochs
    train = args.train
    filter = args.filter
    flat = args.flat

    # load dataset
    train_dataset, val_dataset = load_dataset(validation_split=validation_split, datasets_path=datasets_path, count=total_files, filter=filter, flat=flat)
   
    visualize_raw = args.raw

    if visualize_raw:
        visualize_dataset(train_dataset)    
        exit()

    if train:
        # train and store model
        criterion = footstep_loss
        train_store(train_dataset, val_dataset, batch_size=batch_size, epochs=epochs, criterion=criterion, model_path=model_path)

    else:
        # load and validate model
        load_validate(train_dataset, batch_size=1, model_path=model_path)


    torch.cuda.empty_cache()



