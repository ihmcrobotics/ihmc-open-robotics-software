import h5py
import cv2
import numpy as np
import os
from h5py import Group, Dataset
import matplotlib.pyplot as plt



def get_data(data, namespace):
    ds = []

    print(data[namespace].keys())

    keys = list(map(str, sorted(list(map(int, data[namespace].keys())))))

    for key in keys:
        print('Appending: {}'.format(namespace + key))
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

            channels.append(dict(name=group, count=len(data[group].keys()), dtype='byte'))

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
        dtype = data[group + '/0'][:].dtype

    print(f"{'  ' + group:<45} {str(total_groups):<20} {str(total_datasets):<25} {str(dtype):<10}")

    # print(f"{'Groups: ' + str(len(groups)):<25} Datasets: {str(len(datasets))}")

def get_data(data, namespace):
    ds = []

    print(data[namespace].keys())

    keys = list(map(str, sorted(list(map(int, data[namespace].keys())))))

    for key in keys:
        print('Appending: {}'.format(namespace + key))
        array = data[namespace + key][:]
        ds.append(array)

    data_block = np.vstack(ds)
    return data_block

def display_image(data, index, namespace, delay):
    buffer = data[namespace + str(index)][:].byteswap().view('uint8')
    buffer_image = np.asarray(buffer, dtype=np.uint8)
    buffer_image = cv2.imdecode(buffer_image, cv2.IMREAD_GRAYSCALE)
    cv2.imshow("Depth Image", buffer_image)
    code = cv2.waitKeyEx(delay)

    if code == 113:
        exit()

def playback_images(data, channels):

    for i in range(len(data[namespace].keys())):

        for channel in channels:
            display_image(data, channel)
        
        code = cv2.waitKeyEx(30)
        if code == 113:
            exit()

def load_file(file_name):

    path = '/home/quantum/.ihmc/logs/perception/'

    files = ['20230216_140029_PerceptionLog.hdf5']

    for i, file in enumerate(files):
        print("File:", i, files[i])

    data = h5py.File(path + files[0], 'r')

def plot_position(data_list, style_list, tag, type_string):


    fig, axs = plt.subplots(3, figsize=(30,10))
    fig.suptitle(tag + ' ' + type_string + ' Plots')

    axs[0].set_title(tag + ' ' + type_string + ' (X)')
    axs[1].set_title(tag + ' ' + type_string + ' (Y)')
    axs[2].set_title(tag + ' ' + type_string + ' (Z)')

    # axs[0].set_ylim(-4, 4)
    # axs[1].set_ylim(-4, 4)
    # axs[2].set_ylim(-4, 4)

    for i, data in enumerate(data_list):
        
        t = np.linspace(0, data.shape[0], data.shape[0])
        axs[0].plot(t, data[:,0], style_list[i], markersize=1)
        axs[1].plot(t, data[:,1], style_list[i], markersize=1)
        axs[2].plot(t, data[:,2], style_list[i], markersize=1)

    plt.show()