import os, timeit

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from perception_msgs.msg import DetectedObjectPacket

from lib.opts import opts
from lib.detectors.object_pose import ObjectPoseDetector

from perception_msgs.msg import ImageMessage

import cv2
import time

# def cube_vertices(Xc,Yc,Zc,SL):
#     return [[Xc + SL/2, Yc + SL/2, Zc + SL/2],
#             [Xc + SL/2, Yc + SL/2, Zc - SL/2],
#             [Xc + SL/2, Yc - SL/2, Zc + SL/2],
#             [Xc + SL/2, Yc - SL/2, Zc - SL/2],
#             [Xc - SL/2, Yc + SL/2, Zc + SL/2],
#             [Xc - SL/2, Yc + SL/2, Zc - SL/2],
#             [Xc - SL/2, Yc - SL/2, Zc + SL/2],
#             [Xc - SL/2, Yc - SL/2, Zc - SL/2]]

class Image2CenterPose_node(Node):
    def __init__(self):
        super().__init__('zed2_centerpose_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Default params with commandline input
        self.opt = opts().parser.parse_args()

        self.opt.arch = 'dlav1_34'
        # self.opt.load_model = f"/root/centerpose-ros2/models/chair_v1_140.pth"
        self.opt.load_model = f"/root/centerpose-ros2/models/cup_mug_v1_140.pth"
        self.opt.debug = 5

        # Default setting
        self.opt.nms = True
        self.opt.obj_scale = True
        self.image_process_frequency = 5
        self.image_process_period_ns = int(1e9 / self.image_process_frequency)
        self.last_image_process_time_ns = 0

        # Tracking stuff
        if self.opt.tracking_task == True:
            print('Running tracking')
            self.opt.pre_img = True
            self.opt.pre_hm = True
            self.opt.tracking = True
            self.opt.pre_hm_hp = True
            self.opt.tracking_hp = True
            self.opt.track_thresh = 0.1

            self.opt.obj_scale_uncertainty = True
            self.opt.hps_uncertainty = True
            self.opt.kalman = True
            self.opt.scale_pool = True

            self.opt.vis_thresh = max(self.opt.track_thresh, self.opt.vis_thresh)
            self.opt.pre_thresh = max(self.opt.track_thresh, self.opt.pre_thresh)
            self.opt.new_thresh = max(self.opt.track_thresh, self.opt.new_thresh)

            # # For tracking moving objects, better to set up a small threshold
            # opt.max_age = 2

            print('Using tracking threshold for out threshold!', self.opt.track_thresh)

        # PnP related
        self.meta = {}
        if self.opt.cam_intrinsic is None:
            self.meta['camera_matrix'] = np.array(
                # [[663.0287679036459, 0, 300.2775065104167], [0, 663.0287679036459, 395.00066121419275], [0, 0, 1]])
                # [[261.0508728027344, 0, 316.148193359375], [0, 261.0508728027344, 181.02622985839844], [0, 0, 1]]) # zed
                # [[261.0508728027344, 0, 316.148193359375], [0, 261.0508728027344, 181.02622985839844], [0, 0, 1]]) # iphone
                [[1.62160860e+03, 0.00000000e+00, 6.40337225e+02], [0.00000000e+00, 1.64687719e+03, 9.68022092e+02], [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
            self.opt.cam_intrinsic = self.meta['camera_matrix']
        else:
            self.meta['camera_matrix'] = np.array(self.opt.cam_intrinsic).reshape(3, 3)

        self.opt.use_pnp = True

        # Update default configurations
        self.opt = opts().parse(self.opt)

        # Update dataset info/training params
        self.opt = opts().init(self.opt)

        if self.opt.use_pnp == True and 'camera_matrix' not in self.meta.keys():
            raise RuntimeError('Error found. Please give the camera matrix when using pnp algorithm!')

        os.environ['CUDA_VISIBLE_DEVICES'] = self.opt.gpus_str
        self.opt.debug = max(self.opt.debug, 1)
        # Detector = detector_factory[self.opt.task]
        # self.detector = Detector(self.opt)
        self.detector = ObjectPoseDetector(self.opt)
        self.detector.pause = False

        self.subscription = self.create_subscription(
            ImageMessage,
            '/ihmc/zed2/left_color',
            self.listener_callback,
            qos_profile)

        # Create a publisher for the pose topic
        self.centerpose_publisher_ = self.create_publisher(DetectedObjectPacket, '/ihmc/centerpose', 1)
        self.detected_object = DetectedObjectPacket()

        self.rot_mat = Rotation.from_euler('xyz', [np.deg2rad(90.0), 0.0, np.deg2rad(90.0)]).as_matrix()
        print(self.rot_mat)

        self.get_logger().info("Waiting for an Image...")
        self.start_time = timeit.default_timer()

        self.doOnce = True

    def listener_callback(self, msg):
        # Skip the ImageMessage if not enough time has passed since we processed the last one to save CPU - it can't
        # keep up at 30hz even on an i7 13700 -danderson
        if time.time_ns() - self.last_image_process_time_ns < self.image_process_period_ns:
            return
        self.last_image_process_time_ns = time.time_ns()

        self.get_logger().info("Processing ImageMessage #" + str(msg.sequence_number))
        image_np = np.frombuffer(b''.join(msg.data), dtype=np.uint8)
        image = cv2.imdecode(image_np, cv2.COLOR_YUV2RGB)
        if(self.doOnce):
            cv2.imwrite('image.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 50])
            self.doOnce = False
        ret = self.detector.run(np.asarray(image), meta_inp=self.meta)

        if len(ret['results']) > 0:
            current_time = timeit.default_timer()

            # self.detected_object.pose.position.x = 0.5*np.sin(0.5*(self.start_time - current_time))
            # self.detected_object.pose.position.y = 0.5*np.cos(0.5*(self.start_time - current_time))
            # self.detected_object.pose.position.z = 0.0
            # self.detected_object.pose.orientation.x = 0.0
            # self.detected_object.pose.orientation.y = 0.0
            # self.detected_object.pose.orientation.z = 0.0
            # self.detected_object.pose.orientation.w = 1.0
            # nn_out_vertices = cube_vertices(0.5*np.sin(0.5*(self.start_time - current_time)),
            #                          0.5*np.cos(0.5*(self.start_time - current_time)),
            #                          0,
            #                          1*np.sin(0.5*(self.start_time - current_time)))
            
            # if 'kps_3d_cam' not in ret['results'][0]: 
            #     print(ret)
                
            if 'kps_3d_cam' in ret['results'][0]: 
                nn_out_vertices = ret['results'][0]['kps_3d_cam'] # shape 9x3 (1st row is object centroid location)
                point_verts = []
                for vertex in nn_out_vertices:
                    point = Point()
                    point.x = vertex[0]
                    point.y = vertex[1]
                    point.z = vertex[2]
                    point_verts.append(point)
                self.detected_object.bounding_box_vertices = point_verts[-8:]

            if 'projected_cuboid' in ret['results'][0]:
                bbox = ret['results'][0]['projected_cuboid']
                for vertex in bbox:
                    point = Point()
                    point.x = vertex[0]
                    point.y = vertex[1]
                    # point.z = vertex[2]
                    point_verts.append(point)
                self.detected_object.bounding_box_2d_vertices = point_verts[-8:]

            if 'location' in ret['results'][0]: 
                self.detected_object.pose.position.x = ret['results'][0]['location'][0]/1000.0
                self.detected_object.pose.position.y = ret['results'][0]['location'][1]/1000.0
                self.detected_object.pose.position.z = ret['results'][0]['location'][2]/1000.0
                self.detected_object.pose.orientation.x = ret['results'][0]['quaternion_xyzw'][0]
                self.detected_object.pose.orientation.y = ret['results'][0]['quaternion_xyzw'][1]
                self.detected_object.pose.orientation.z = ret['results'][0]['quaternion_xyzw'][2]
                self.detected_object.pose.orientation.w = ret['results'][0]['quaternion_xyzw'][3]

            self.detected_object.confidence = self.start_time - current_time
            self.detected_object.object_type = "cup"
            
            self.get_logger().info('Object Detected in the Image!')
            self.centerpose_publisher_.publish(self.detected_object)

def main(args=None):
    # Check if models directory exists
    # Download pretrained models from here https://drive.google.com/drive/folders/16HbCnUlCaPcTg4opHP_wQNPsWouUlVZe
    if not os.path.exists('models'):
        print('Could not find models directory')
        return

    rclpy.init(args=args)

    node = Image2CenterPose_node()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()