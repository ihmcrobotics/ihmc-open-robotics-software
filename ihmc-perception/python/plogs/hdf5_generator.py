import h5py
import numpy as np
from hdf5_reader import *

def insert_image_datasets(data, dataset_paths, group_names):

    for index, dataset_path in enumerate(dataset_paths):
        insert_compressed_images(data, dataset_path, group_names[index])    

    return data


def insert_compressed_images(data, dataset_path, group):

    image_files = sorted(os.listdir(dataset_path))

    for index, image_file in enumerate(image_files):
            
        image = cv2.imread(dataset_path + image_file, cv2.IMREAD_ANYCOLOR)    

        cv2.imshow('Image', image)
        code = cv2.waitKey(1)
        if code == ord('q'):
            break


        encoded_img = np.array((cv2.imencode('.jpg', image))[1])
        
        print(encoded_img)
        data.create_dataset(group + str(index), shape=encoded_img.shape, data=encoded_img)
    
        
def insert_timestamps(data, timestamps_path, group):

    timestamps_file = open(timestamps_path, 'r')

    timestamps = []
    for line in timestamps_file:
        timestamps.append(float(line))
    timestamps = np.array(timestamps, dtype=np.float32)

    data.create_dataset(group + 'time', shape=timestamps.shape, data=timestamps)

    timestamps_file.close()

    return data

def insert_poses(data, poses_path, group):

    poses_file = open(poses_path, 'r')

    poses = []
    for line in poses_file:
        poses.append([float(x) for x in line.split()])
    poses = np.array(poses, dtype=np.float32)

    print(poses.shape)

    data.create_dataset(group + 'poses', shape=poses.shape, data=poses)

    poses_file.close()

    return data