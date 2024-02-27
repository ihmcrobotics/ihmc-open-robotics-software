import h5py
import cv2
import numpy as np
import os
from h5py import Group, Dataset
import matplotlib.pyplot as plt

def get_data(data, namespace):
    ds = []

    # print(data[namespace].keys())

    keys = list(map(str, sorted(list(map(int, data[namespace].keys())))))

    for key in keys:
        # print('Appending: {}'.format(namespace + key))
        array = data[namespace + key][:]
        ds.append(array)

    data_block = np.vstack(ds)
    return data_block

def collect_groups(data):
    groups = []
    topics = []
    data.visit(topics.append)

    for topic in topics:
        if isinstance(data[topic], Group):
            groups.append(topic + '/')

    return groups

def collect_datasets(data):
    datasets = []
    topics = []
    data.visit(topics.append)

    for topic in topics:
        if isinstance(data[topic], Dataset):
            datasets.append(topic)

    return datasets

def print_file_info(h5, h5_filename):

    print('\n\n------------------------------------------ HDF5 File Info --------------------------------------------\n\n')
    print('File: {}\n'.format(h5_filename))
    print(f"{'Group ':<45} {'Total Groups ':<20} {'Total Datasets ':<25} {'Data Type ':<10}")
    print()

    groups = collect_groups(h5)

    for group in groups:
        print_group_info(h5, group)

    print('------------------------------------------------------------------------------------------------------')
    print()

def print_file_size(path, filename):
    print(f"{'  ' + filename:<45} {os.path.getsize(path + filename) / (1024 * 1024):<8.3f} MB")

def print_file_sizes(path, filenames):    
    print('------------------------------------------ HDF5 File Sizes --------------------------------------------\n\n')
    print(f"{'File Name':<45} {'File Size (MB)':<20}")
    print()

    for filename in filenames:
        if filename.endswith('.hdf5'):
            print_file_size(path, filename)

    print('------------------------------------------------------------------------------------------------------')
    print()

def rename_file(path, h5, h5_filename):
    # ensures the only modded files are the old 20230207 file names
    if(h5_filename[0].isdigit() and h5_filename[1].isdigit() and h5_filename[2].isdigit() and "plog" not in h5_filename):
        print("Old file name: " + h5_filename)
        unique_set = set({}) # unique set of all base groups in the HD5F file
        groups = collect_groups(h5)

        # collect the unique base groups
        for group in groups:
            stringSplit = group.split("/") 
            unique_set.add(stringSplit[0]) 

        # replace file name using unique groups
        newFileName = h5_filename.replace("PerceptionLog", "plog") 
        stringSplit = newFileName.split("_") 
        newFileName = stringSplit[0] + "_" + stringSplit[1] + "_"
        newFileName += ''.join( s.capitalize()+"_" for s in unique_set) # replace time stamp with sensor usage info
        for i in range(2, len(stringSplit)):
            newFileName += stringSplit[i]
            if(i < len(stringSplit)-1):
                newFileName += "_"
        
        # rename the file if user likes the change
        print("New file name: " + newFileName)
        answer = input("Confirm the name change?: [y/n] ")
        if(answer.lower() == 'y' or 'yes'):
            oldFile = path + h5_filename
            newFileName = path + newFileName
            os.rename(src=oldFile, dst=newFileName) # rename file
            return True
        else:
            print("Name unchanged.")
            return False
    else:
        return False

# rename all files in .ihmc/logs/perception that use the old date_time_Perceptionlog.hd5f format
def rename_all_files(path, filenames):
    unTouchedFiles = []

    for filename in filenames:
        if filename.endswith('.hdf5'):
            h5= h5py.File(path + filename, 'r')
            if(not rename_file(path, h5, filename)):
                unTouchedFiles.append(filename)

    print("Files that were not changed:")
    print(unTouchedFiles)

def collect_channels(data):
    channels = []

    groups = collect_groups(data)

    for group in groups:
        print("Group: ", group)
        if len(data[group].keys()) > 10:
            print("Adding Channel: ", group, " Count: ", len(data[group].keys()))

            type = 'none'

            float_types = ['position', 'orientation', 'time']
            byte_types = ['image', 'depth', 'color']

            if any(x in group for x in float_types):
                type = 'float'
            elif any(x in group for x in byte_types):
                type = 'byte'

            channels.append(dict(name=group, count=len(data[group].keys()), dtype=type))

    return channels


def print_group_info(data, group):

    total_groups = 0
    total_datasets = 0

    for key in data[group].keys():
        if isinstance(data[group + key], Group):
            total_groups += 1
        elif isinstance(data[group + key], Dataset):
            total_datasets += 1

    dtype = 'none'

    if total_datasets > 0:
        dtype = data[group + '/' + list(data[group].keys())[0]][:].dtype

    print(f"{'  ' + group:<45} {str(total_groups):<20} {str(total_datasets):<25} {str(dtype):<10}")

    # print(f"{'Groups: ' + str(len(groups)):<25} Datasets: {str(len(datasets))}")

def get_data(data, namespace):
    ds = []

    # print(data[namespace].keys())

    keys = list(map(str, sorted(list(map(int, data[namespace].keys())))))

    for key in keys:
        # print('Appending: {}'.format(namespace + key))
        array = data[namespace + key][:]
        ds.append(array)

    data_block = np.vstack(ds)
    return data_block

def load_depth(data, index, namespace):
    buffer = data[namespace + str(index)][:].byteswap().view('uint8')
    buffer_image = np.asarray(buffer, dtype=np.uint8)
    buffer_image = cv2.imdecode(buffer_image, cv2.IMREAD_UNCHANGED)  # Use cv2.IMREAD_UNCHANGED
    if buffer_image.dtype != np.uint16:  # Check if it's not already 16-bit
        buffer_image = buffer_image.astype(np.uint16)
    return buffer_image

def load_image(data, index, namespace):
    buffer = data[namespace + str(index)][:].byteswap().view('uint8')
    buffer_image = np.asarray(buffer, dtype=np.uint8)
    buffer_image = cv2.imdecode(buffer_image, cv2.IMREAD_ANYDEPTH)
    return buffer_image

def display_image(data, index, namespace, delay, name="Depth Image"):
    buffer = data[namespace + '/' + str(index)][:].byteswap().view('uint8')
    buffer_image = np.asarray(buffer, dtype=np.uint8)
    buffer_image = cv2.imdecode(buffer_image, cv2.IMREAD_GRAYSCALE)

    show_depth(name, buffer_image, delay)
    
def show_depth(name, image, delay):

    # Convert 16-bit height map to 8-bit grayscale for display
    image = cv2.convertScaleAbs(image, alpha=(255.0/65535.0))

    # Make the image brighter
    image = np.minimum(image * 10, 255)

    # # Remove spikes from image by thresholding
    # buffer_image[buffer_image > 80] = 0

    # # Normalize image using OpenCV
    # buffer_image = cv2.normalize(buffer_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)


    cv2.imshow(name, buffer_image)
    code = cv2.waitKeyEx(delay)

    # print("Code: ", code)

    if code == 113 or code == 1048689:
        exit()

def playback_images(data, channels):

    for i in range(len(data[namespace].keys())):

        for channel in channels:
            display_image(data, channel)
        
        code = cv2.waitKeyEx(30)
        if code == 113 or code == 1048689:
            exit()

def list_files():
    home = os.path.expanduser('~')
    path = home + '/.ihmc/logs/perception/'
    files = sorted(os.listdir(path))
    files = [file for file in files if file[-5:] == '.hdf5']

    for i, file in enumerate(files):
        print("File:", i, files[i])

def load_file(file_name):
    home = os.path.expanduser('~')
    path = home + '/.ihmc/logs/perception/'
    data = h5py.File(path + file_name, 'r')
    return data

def plot_position(start_index, end_index, data_list, style_list, tag, type_string):

    font = {'family' : 'normal',
        'size'   : 20}

    import matplotlib
    matplotlib.rc('font', **font)

    # Compute RMSE between data_list[0] and data_list[1] with Eucledian distance between start index and end index
    fig3 = plt.figure(constrained_layout=True, figsize=(17,7))
    gs = fig3.add_gridspec(3, 7)
    xt_ax = fig3.add_subplot(gs[0, :4])
    xt_ax.set_title('Position (X) vs Keyframe Index')
    yt_ax = fig3.add_subplot(gs[1, :4])
    yt_ax.set_title('Position (Y) vs Keyframe Index')
    zt_ax = fig3.add_subplot(gs[2, :4])
    zt_ax.set_title('Position (Z) vs Keyframe Index')
    xy_ax = fig3.add_subplot(gs[:3, 4:7])
    xy_ax.set_title('Position (X) vs Position (Y)')

    # xt_ax.set_title(tag + ' ' + type_string + ' (X)')
    # yt_ax.set_title(tag + ' ' + type_string + ' (Y)')
    # zt_ax.set_title(tag + ' ' + type_string + ' (Z)')
    # xy_ax.set_title(tag + ' ' + type_string + ' (X-Y)')

    xt_ax.set_xlabel('Keyframe Index')
    yt_ax.set_xlabel('Keyframe Index')
    zt_ax.set_xlabel('Keyframe Index')
    xy_ax.set_xlabel('X-Position (m)')

    xt_ax.set_ylabel('X-Position (m)')
    yt_ax.set_ylabel('Y-Position (m)')
    zt_ax.set_ylabel('Z-Position (m)')
    xy_ax.set_ylabel('Y-Position (m)')

    for i, data in enumerate(data_list):
        
        if not(end_index == -1):
            t = np.linspace(start_index, end_index, end_index - start_index)
        else:
            t = np.linspace(start_index, data.shape[0], data.shape[0] - start_index -1)

        xt_ax.plot(t, data[start_index : end_index,0], style_list[i], markersize=1)
        yt_ax.plot(t, data[start_index : end_index,1], style_list[i], markersize=1)
        zt_ax.plot(t, data[start_index : end_index,2], style_list[i], markersize=1)
        xy_ax.plot(data[start_index : end_index,0], data[start_index : end_index,1], style_list[i], markersize=1)


    # Add a legend on the top-left corner with blue being Ground Truth and Red being the estimated position
    # xt_ax.legend(['Ground Truth', 'Sensor Position'], loc='upper left')
    # yt_ax.legend(['Ground Truth', 'Sensor Position'], loc='upper left')
    # zt_ax.legend(['Ground Truth', 'Sensor Position'], loc='upper left')
    xy_ax.legend(['Ground Truth', 'Sensor Position'], loc='upper left')


    plt.show()

def load_raw_height_maps(data, dataset_name):
    height_map = data[dataset_name][:]
    return height_map

def show_height_map(height_map, delay):
    height_map_display = height_map.copy()

    # convert grayscale to RGB
    height_map_display = cv2.cvtColor(height_map_display, cv2.COLOR_GRAY2RGB)

    # Resize the height map to 1000x1000
    height_map_display = cv2.resize(height_map_display, (1000, 1000))

    # compute scale factor
    cv2.imshow("Height Map", height_map_display)
    code = cv2.waitKeyEx(delay)

    if code == 113 or code == 1048689:
        exit()
    
def load_height_maps(data, count):
    height_maps = []    
    total_height_maps = len(data.keys()) if len(data.keys()) < count else count
    # for i in range(total_height_maps):
    height_map = load_raw_height_maps(data, "matrix")

    # pad with 0s if less than 201 x 201
    if height_map.shape[0] < 201:
        height_map = np.pad(height_map, (0, 201 - height_map.shape[0]), 'constant')

    print("Height Map Shape: ", height_map.shape)

    height_maps.append(height_map)
    return height_maps