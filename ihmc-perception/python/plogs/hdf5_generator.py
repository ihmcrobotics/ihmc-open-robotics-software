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
    
        
def insert_compressed_depth_maps(data, dataset_path, group, display=False):

    image_files = sorted(os.listdir(dataset_path))

    for index, image_file in enumerate(image_files):
            
        # load bytes from PNG file into numpy array
        image = cv2.imread(dataset_path + image_file, cv2.IMREAD_ANYDEPTH)

        print("Depth Shape: ", image.shape)

        if display:
            cv2.imshow('Image', image)
            code = cv2.waitKey(1)
            if code == ord('q'):
                break

        encoded_img = np.array((cv2.imencode('.png', image))[1])
        data.create_dataset(group + str(index), shape=encoded_img.shape, data=encoded_img)

def insert_timestamps(data, timestamps_path, group):

    timestamps_file = open(timestamps_path, 'r')

    timestamps = []
    for line in timestamps_file:
        if line.split(' ')[0] != '#':
            words = line.split(' ')
            print("Time Found: ", float(words[0]))
            timestamps.append(float(words[0]))

    timestamps = np.array(timestamps, dtype=np.float64)

    data.create_dataset(group + 'time', shape=timestamps.shape, data=timestamps)

    np.set_printoptions(suppress=True)
    print("Times: ", timestamps.tolist())
    np.set_printoptions(suppress=False)

    timestamps_file.close()

    return data

def insert_poses(data, poses_path, group):

    poses_file = open(poses_path, 'r')

    poses = []
    for line in poses_file:
        poses.append([float(x) for x in line.split()])
    poses = np.array(poses, dtype=np.float64)

    print(poses.shape)

    data.create_dataset(group + 'poses', shape=poses.shape, data=poses)

    poses_file.close()

    return data

def insert_poses_quaternion(data, poses_path, group):

    poses_file = open(poses_path, 'r')

    times = []
    tvecs = []
    qvecs = []
    for line in poses_file:
        words = line.split()
        if words[0] != '#':
            times.append(float(words[0]))
            tvecs.append([float(x) for x in words[1:4]])
            qvecs.append([float(x) for x in words[4:8]])
    
    times = np.array(times, dtype=np.float64)
    poses_tvec = np.array(tvecs, dtype=np.float64)
    poses_qvec = np.array(qvecs, dtype=np.float64)

    print(poses_tvec.shape, poses_qvec.shape)

    data.create_dataset(group + 'time', shape=times.shape, data=times)
    data.create_dataset(group + 'position', shape=poses_tvec.shape, data=poses_tvec)
    data.create_dataset(group + 'orientation', shape=poses_qvec.shape, data=poses_qvec)

    poses_file.close()

    return data

def resample_poses_quaternion(data, src_grp, dst_grp, time_group, block_size=10):
    # find the nearest position and quaternion for each time
    # and insert into the dataset

    times = data[time_group + 'time'][:]

    print(times)

    poses_tvec = data[src_grp + 'position']
    poses_qvec = data[src_grp + 'orientation']

    poses_tvec_resampled = []
    poses_qvec_resampled = []

    for time in times:
        index = np.argmin(np.abs(data[src_grp + 'time'][:] - time))
        gt_time = data[src_grp + 'time'][index]
        poses_tvec_resampled.append(poses_tvec[index])
        poses_qvec_resampled.append(poses_qvec[index])

        print("Index Closest: ", index, "Time: ", time, "GT Time: ", gt_time)

    poses_tvec_resampled = np.array(poses_tvec_resampled, dtype=np.float64)
    poses_qvec_resampled = np.array(poses_qvec_resampled, dtype=np.float64)

    # create datasets of size block_size each
    for i in range(0, len(times), block_size):
        data.create_dataset(dst_grp + 'position/' + str(int(i/10)), shape=poses_tvec_resampled[i:i+block_size].shape, data=poses_tvec_resampled[i:i+block_size])
        data.create_dataset(dst_grp + 'orientation/' + str(int(i/10)), shape=poses_qvec_resampled[i:i+block_size].shape, data=poses_qvec_resampled[i:i+block_size])


def log_height_maps(data, height_maps, dataset):
    index = 0
    for height_map in height_maps:
        log_height_map(data, height_map, dataset + str(index))
        index += 1

def log_height_map(data, height_map, dataset):

    height_map = (height_map - 3.2768) * 10000
    height_map = np.array(height_map).astype(np.uint16)

    encoded_img = np.array((cv2.imencode('.png', height_map))[1])
    data.create_dataset(dataset, shape=encoded_img.shape, data=encoded_img)

    print("Written: ", dataset, height_map.shape, encoded_img.shape)



