import h5py
import cv2
import numpy as np
import os

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

def get_topic_list(data)
    names = []

    name = ''

    for key in len(data.keys())
        recursively_explore(key, name);

    return names;

def recursively_explore(data, key, name):
    


if __name__ == '__main__':

    path = '/home/bmishra/Downloads/'
    file = '20230113_145054_Images.hdf5'

    data = h5py.File(path + file, 'r')

    image_topic_1 = '/image/'

    depth_topic_l515 = '/l515/depth'
    color_topic_l515 = '/l515/color'

    for i in range(len(data[image_topic_1].keys())):

        color = data[image_topic_1 + str(i)][:].byteswap().view('uint8')
        
        # l515_color = data[color_topic_l515 + str(i)][:].byteswap().view('uint8')
        # l515_depth = data[depth_topic_l515 + str(i)][:].byteswap().view('uint8')

        color_image = np.asarray(color, dtype=np.uint8)
        # color_image = np.asarray(color, dtype=np.uint8)
        # depth_image = np.asarray(depth, dtype=np.uint8)

        # use imdecode function
        color_image = cv2.imdecode(color_image, cv2.IMREAD_COLOR)
        # depth_image = cv2.imdecode(depth_image, cv2.IMREAD_COLOR)

        print("Image: {}".format(i))

        cv2.imshow("color_image", color_image)
        # cv2.imshow("Depth Image", depth_image)
        code = cv2.waitKeyEx(30)
        if code == 113:
            exit()