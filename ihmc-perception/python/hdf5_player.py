import h5py
import cv2
import numpy as np
import os
from h5py import Group, Dataset

from hdf5_converter import *

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

def collect_groups(data, topics):
    groups = []

    for topic in topics:
        if isinstance(data[topic], Group):
            groups.append(topic)

    return groups

def collect_datasets(data, topics):
    datasets = []

    for topic in topics:
        if isinstance(data[topic], Dataset):
            datasets.append(topic)

    return datasets

def print_file_info(h5, h5_filename):

    print('\n\n------------------------------------------ HDF5 File Info --------------------------------------------\n\n')
    print('File: {}\n'.format(h5_filename))
    print(f"{'Group ':<45} {'Total Groups ':<20} {'Total Datasets ':<25} {'Data Type ':<10}")
    print()

    topics = []

    h5.visit(topics.append)

    groups = collect_groups(h5, topics)

    for group in groups:
        print_group_info(h5, group)

    print('------------------------------------------------------------------------------------------------------')
    print()

def print_group_info(data, group):

    topics = [group + "/" + key for key in data[group].keys()]


    groups = collect_groups(data, topics)
    datasets = collect_datasets(data, topics)

    dtype = 'none'

    if len(datasets) > 0:
        dtype = data[datasets[0]][:].dtype

    print(f"{'  ' + group:<45} {str(len(groups)):<20} {str(len(datasets)):<25} {str(dtype):<10}")

    # print(f"{'Groups: ' + str(len(groups)):<25} Datasets: {str(len(datasets))}")

if __name__ == '__main__':

    path = '/home/bmishra/.ihmc/logs/perception/'

    old_file = '20230207_214209_PerceptionLog.hdf5'

    new_file = '20230217_130000_PerceptionLog.hdf5'

    test_file = 'UrbanIndoorPlanesOuster.hdf5'

    new_h5 = h5py.File(path + new_file, 'r')

    old_h5 = h5py.File(path + old_file, 'r')

    test_h5 = h5py.File(path + test_file, 'w')

    # topics = get_topic_list(data)


    print_file_info(old_h5, old_file)
    # print_file_info(new_h5, new_file)

    for i in range(507):
        copy_byte_dataset(old_h5, "ouster/depth/" + str(i), test_h5, "ouster/depth/" + str(i))

    for i in range(50):
        copy_float_dataset(old_h5, "ouster/sensor/position/" + str(i), test_h5, "ouster/sensor/position/" + str(i))

    for i in range(50):
        copy_float_dataset(old_h5, "ouster/sensor/orientation/" + str(i), test_h5, "ouster/sensor/orientation/" + str(i))

    print_file_info(test_h5, test_file)

    test_h5.close()

    # image_topic_1 = '/image/'

    # depth_topic_l515 = '/l515/depth'
    # color_topic_l515 = '/l515/color'

    # for i in range(len(data[image_topic_1].keys())):

    #     color = data[image_topic_1 + str(i)][:].byteswap().view('uint8')
        
    #     # l515_color = data[color_topic_l515 + str(i)][:].byteswap().view('uint8')
    #     # l515_depth = data[depth_topic_l515 + str(i)][:].byteswap().view('uint8')

    #     color_image = np.asarray(color, dtype=np.uint8)
    #     # color_image = np.asarray(color, dtype=np.uint8)
    #     # depth_image = np.asarray(depth, dtype=np.uint8)

    #     # use imdecode function
    #     color_image = cv2.imdecode(color_image, cv2.IMREAD_COLOR)
    #     # depth_image = cv2.imdecode(depth_image, cv2.IMREAD_COLOR)

    #     print("Image: {}".format(i))

    #     cv2.imshow("color_image", color_image)
    #     # cv2.imshow("Depth Image", depth_image)
    #     code = cv2.waitKeyEx(30)
    #     if code == 113:
    #         exit()