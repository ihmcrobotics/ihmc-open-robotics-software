import h5py
import cv2
import numpy as np
import os

# 20221212_184748_PerceptionLog.hdf5
# 20221212_184906_PerceptionLog.hdf5
# 20221212_184940_PerceptionLog.hdf5

# 20221216_141954_PerceptionLog.hdf5
# 20221216_143619_PerceptionLog.hdf5
# 20221216_144027_PerceptionLog.hdf5


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

if __name__ == '__main__':

    path = '/home/bmishra/Workspace/Data/Sensor_Logs/Depth/Good/'

    files = os.listdir(path)
    files = sorted(files)

    # good_files = [files[i] for i in [7, 8, 9, 14, 20, 22]]

    for i, file in enumerate(files):
        print("File:", i, files[i])

    # Good logs
    # [7, 8, 9, 14, 20, 22]

    data = h5py.File(path + files[1], 'r')

    position_data = get_data(data, '/l515/sensor/position/')



    print(position_data.shape)

    print(position_data)


    for i in range(len(data['/l515/depth/'].keys())):

        # color = data['/l515/color/' + str(i)][:].byteswap().view('uint8')
        depth = data['/l515/depth/' + str(i)][:].byteswap().view('uint8')
        # img = cv2.imdecode()

        # print(color[-10:])

        # print("Shape: ", color.shape, " DType:", color.dtype)

        # color_image = np.asarray(color, dtype=np.uint8)
        depth_image = np.asarray(depth, dtype=np.uint8)


        # use imdecode function
        # color_image = cv2.imdecode(color_image, cv2.IMREAD_COLOR)
        depth_image = cv2.imdecode(depth_image, cv2.IMREAD_COLOR)

        print("Image: {}".format(i))

        # cv2.imshow("color_image", color_image)
        cv2.imshow("Depth Image", depth_image)
        code = cv2.waitKeyEx(30)
        if code == 113:
            exit()