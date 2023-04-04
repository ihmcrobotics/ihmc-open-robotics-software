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

def display_image(data, index, namespace, delay):
    buffer = data[namespace + str(index)][:].byteswap().view('uint8')
    buffer_image = np.asarray(buffer, dtype=np.uint8)
    buffer_image = cv2.imdecode(buffer_image, cv2.IMREAD_GRAYSCALE)
    
    # print('Image: ', buffer_image.dtype, buffer_image.shape)

    # Make the image brighter
    buffer_image = np.minimum(buffer_image * 10, 255)

    # # Remove spikes from image by thresholding
    # buffer_image[buffer_image > 80] = 0

    # # Normalize image using OpenCV
    # buffer_image = cv2.normalize(buffer_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)


    cv2.imshow("Depth Image", buffer_image)
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

def load_file(file_name):

    path = '/home/quantum/.ihmc/logs/perception/'

    files = ['20230216_140029_PerceptionLog.hdf5']

    for i, file in enumerate(files):
        print("File:", i, files[i])

    data = h5py.File(path + files[0], 'r')

def plot_position(start_index, end_index, data_list, style_list, tag, type_string):


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
    xt_ax.legend(['Ground Truth', 'SKIPR Position'], loc='upper left')
    yt_ax.legend(['Ground Truth', 'SKIPR Position'], loc='upper left')
    zt_ax.legend(['Ground Truth', 'SKIPR Position'], loc='upper left')
    xy_ax.legend(['Ground Truth', 'SKIPR Position'], loc='upper left')


    plt.show()