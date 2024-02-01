import rosbag
import h5py
import sys
import ros_numpy
import numpy as np
import cv2

def rosbag_info(filename):
    bag = rosbag.Bag(filename)
    print(bag._get_yaml_info())

def convert_pc_msg_to_np(pc_msg):
    # Conversion from PointCloud2 msg to np array.
    pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_msg, remove_nans=True)
    return pc_np

def convert(filename, h5_filename, topics):
    bag = rosbag.Bag(filename)

    data = h5py.File(h5_filename, 'w')
    
    print("Writing to File:", h5_filename)

    for topicname in topics:
        grp = data.create_group(topicname)

    # data.create_dataset('')

    # Use ROSBag Fixer for any L515 message types that produce errors of type MsgNotFound
    # https://github.com/gavanderhoorn/rosbag_fixer

    for topic in topics:

        count = 0

        timestamps = []

        for _, msg, t in bag.read_messages([topic]):

            timestamps.append(t.to_sec())

            if 'sensor_msgs/Image' == msg._type:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
                data.create_dataset(topic + '/' + str(count), shape=(msg.height, msg.width, 2), data=img)
                count += 1

                print("Image: {}\t{}\t{}".format(topic, t.to_sec(), img.shape, count))
                

            if 'sensor_msgs/CompressedImage' == msg._type:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
                data.create_dataset(topic + '/' + str(count), shape=(msg.height, msg.width, 2), data=img)
                count += 1

                print("Image: {}\t{}\t{}".format(topic, t.to_sec(), img.shape, count))

            if 'sensor_msgs/CameraInfo' == msg._type:
                print("CameraInfo", topic, t)  

            if 'sensor_msgs/CompressedImage' == msg._type:
                print("CompressedImage", topic, t)  

            if 'sensor_msgs/PointCloud2' == msg._type:
                print("PointCloud", topic, t)
                pc_np = convert_pc_msg_to_np(msg)
                data.create_dataset(topic + '/' + str(count), shape=pc_np.shape, data=pc_np)
                count += 1

            if 'sensor_msgs/Imu' == msg._type:
                print("IMU", topic, t)

        timestamps = np.array(timestamps, dtype=np.float64)
        data.create_dataset(topic + '/time', shape=timestamps.shape, data=timestamps)


    bag.close()
    data.close()

def play(h5_filename):
    data = h5py.File(h5_filename, 'r')

    n_imgs = len(data['/chest_l515/depth/image_rect_raw'].keys())

    print("Total Images: ", n_imgs)

    for i in range(n_imgs - 2):
        img = data['/chest_l515/depth/image_rect_raw/' + str(i)][:]

        print(img)

        cv2.imshow("Image", img[:,:,0])
        code = cv2.waitKeyEx(30)
        if code == 113:
            exit()

        print("Image Loaded: ", img.shape, code)

    data.close()

if __name__ == "__main__":
    
    user = 'quantum'

    path = '/home/' + user + '/Workspace/Data/Atlas_Logs/ROSBags/'

    bag_filename = path + 'atlas_look_and_step_01_fixed.bag'
    h5_filename = path + 'atlas_perception_run_1.h5'

    if len(sys.argv) > 1:
        if sys.argv[1] == 'convert':
            convert(bag_filename, h5_filename,
                            [
                                '/os_cloud_node/points',
                                # '/os_cloud_node/imu',
                                # '/chest_l515/depth/camera_info',
                                '/chest_l515/depth/image_rect_raw'
                            ])  

        if sys.argv[1] == 'play':
            play(h5_filename)

        if sys.argv[1] == 'rosbag_info':
            rosbag_info(path)


    else:
        print("Provide at least 2 arguments.")


    # /chest_l515/depth/camera_info
    # /chest_l515/depth/image_rect_raw
    # /os_cloud_node/imu
    # /os_cloud_node/points