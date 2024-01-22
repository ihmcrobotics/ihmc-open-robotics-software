import os, time
from typing import List, Optional
from threading import Lock

import cv2
import h5py
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from perception_msgs.msg import DetectedObjectPacket
from perception_msgs.msg import ImageMessage

from lib.opts import opts
from lib.detectors.object_pose import ObjectPoseDetector
from models import CenterPoseModels, archType, experiment_type
from dataclasses import dataclass

@dataclass
class Detection():
    cameraPose : Pose = None
    object_id : int = 0
    stabilizer : int = 0
    sequence_id : int = 0
    skipDetaction : bool = False
    objectType: str = 'object_type'
    kps_2d: List[Point] = None
    kps_2d_array: np.ndarray = np.zeros((3))
    kps_3d: List[Point] = None
    confidence: float = 0.0
    quaternion_xyzw: Rotation = Rotation.from_quat([0, 0, 0, 1])
    position: np.ndarray = np.zeros((3))

@dataclass
class swapRef:
    var1: ImageMessage = None
    var2: ImageMessage = None
    lock: Lock = Lock()

    def set_var1(self, value):
        with self.lock:
            self.var1 = value
            self.swap()

    def get_var2(self):
        with self.lock:
            return self.var2

    def swap(self):
        self.var1, self.var2 = self.var2, self.var1

class ZED2CenterposeNode():
    def __init__(self, experiment:experiment_type):
        self.experiment = experiment

        # Default params with commandline input
        opts_args = opts()
        opts_args.parser.add_argument('--model', type=CenterPoseModels.argtype, default=CenterPoseModels.MUG, choices=CenterPoseModels)
        self.opt = opts_args.parser.parse_args()

        self.my_model = self.opt.model
        self.opt.tracking_task = self.opt.tracking

        self.myModelName = self.my_model.name
        print("Detecting " + self.myModelName)
        self.myModelID = self.my_model.value.getID()
        self.scale = self.my_model.value.getModelScale()

        if self.opt.tracking_task == True:
            print(0)
            self.opt.arch = archType.TRACKING.value
            self.opt.load_model = self.my_model.value.getTrackingModelPath()
        else:
            print(1)
            self.opt.arch = archType.NOTRACKING.value
            self.opt.load_model = self.my_model.value.getModelPath()

        # Default setting
        self.opt.debug = 5
        self.opt.nms = True
        self.opt.obj_scale = True
        self.image_process_frequency = 15
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
            self.opt.max_age = 2

            print('Using tracking threshold for out threshold!', self.opt.track_thresh)

        # PnP related
        self.meta = {}
        if self.opt.cam_intrinsic is None:
            self.meta['camera_matrix'] = np.array(
                [[521.6779174804688, 0, 630.867431640625], [0, 521.6779174804688, 364.546142578125], [0, 0, 1]]) # zed
            self.opt.cam_intrinsic = self.meta['camera_matrix']
        else:
            self.meta['camera_matrix'] = np.array(self.opt.cam_intrinsic).reshape(3, 3)

        self.opt.use_pnp = True

        # Update default configurations
        self.opt = opts_args.parse(self.opt)

        # Update dataset info/training params
        self.opt = opts_args.init(self.opt)

        if self.opt.use_pnp == True and 'camera_matrix' not in self.meta.keys():
            raise RuntimeError('Error found. Please give the camera matrix when using pnp algorithm!')

        os.environ['CUDA_VISIBLE_DEVICES'] = self.opt.gpus_str
        self.opt.debug = max(self.opt.debug, 1)
        self.detector = ObjectPoseDetector(self.opt)
        self.detector.pause = False

        if (self.experiment==experiment_type.LIVE):
            self.ros2_node = Node('zed2_centerpose_node')

            self.depthImageSwapReference = swapRef()

            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1)

            self.subscription = self.ros2_node.create_subscription(
                ImageMessage,
                '/ihmc/zed2/left_color',
                self.listener_color_callback,
                qos_profile)

            self.subscription = self.ros2_node.create_subscription(
                ImageMessage,
                '/ihmc/zed2/depth',
                self.listener_depth_callback,
                qos_profile)

            # Create a publisher for the pose topic
            self.centerpose_publisher_ = self.ros2_node.create_publisher(DetectedObjectPacket, '/ihmc/centerpose', 1)
            self.detected_object = DetectedObjectPacket()

            self.ros2_node.get_logger().info("Waiting for an Image...")

        elif (self.experiment==experiment_type.VIDEO):
            fps = 5

            data = h5py.File('/root/centerpose-ros2/data/20231005_180638_ZEDPerceptionDepthLog.hdf5', 'r')
            depth_imgs = data['zed2']['depth']
            color_imgs = data['zed2']['color']

            for x in range(len(depth_imgs.keys())):
                depthBuffer = depth_imgs[str(x)][:].byteswap().view('uint8')
                depthBufferImage = np.asarray(depthBuffer, dtype=np.uint8)
                depthBufferImage = cv2.imdecode(depthBufferImage, cv2.IMREAD_COLOR)

                colorBuffer = color_imgs[str(x)][:].byteswap().view('uint8')
                colorBufferImage = np.asarray(colorBuffer, dtype=np.uint8)
                colorBufferImage = cv2.imdecode(colorBufferImage, cv2.IMREAD_COLOR)

                detection:Detection = self.processImage(colorBufferImage)

                # if detection is not None and isinstance(detection, Detection):
                #     if detection.skipDetaction == False:
                #         cv2.line(colorBufferImage, tuple(detection.kps_2d_array[0,:]), tuple(detection.kps_2d_array[1,:]), (0, 0, 225), 2)
                #         cv2.line(colorBufferImage, tuple(detection.kps_2d_array[0,:]), tuple(detection.kps_2d_array[2,:]), (0, 0, 225), 2)
                #         cv2.line(colorBufferImage, tuple(detection.kps_2d_array[0,:]), tuple(detection.kps_2d_array[4,:]), (0, 0, 225), 2)
                #         cv2.line(colorBufferImage, tuple(detection.kps_2d_array[1,:]), tuple(detection.kps_2d_array[3,:]), (0, 0, 225), 2)
                #         cv2.line(colorBufferImage, tuple(detection.kps_2d_array[1,:]), tuple(detection.kps_2d_array[5,:]), (0, 0, 225), 2)
                #         cv2.line(colorBufferImage, tuple(detection.kps_2d_array[2,:]), tuple(detection.kps_2d_array[3,:]), (0, 0, 225), 2)
                #         cv2.line(colorBufferImage, tuple(detection.kps_2d_array[2,:]), tuple(detection.kps_2d_array[6,:]), (0, 0, 225), 2)
                #         cv2.line(colorBufferImage, tuple(detection.kps_2d_array[3,:]), tuple(detection.kps_2d_array[7,:]), (0, 0, 225), 2)
                #         cv2.line(colorBufferImage, tuple(detection.kps_2d_array[4,:]), tuple(detection.kps_2d_array[5,:]), (0, 0, 225), 2)
                #         cv2.line(colorBufferImage, tuple(detection.kps_2d_array[4,:]), tuple(detection.kps_2d_array[6,:]), (0, 0, 225), 2)
                #         cv2.line(colorBufferImage, tuple(detection.kps_2d_array[5,:]), tuple(detection.kps_2d_array[7,:]), (0, 0, 225), 2)
                #         cv2.line(colorBufferImage, tuple(detection.kps_2d_array[6,:]), tuple(detection.kps_2d_array[7,:]), (0, 0, 225), 2)

                # img = cv2.hconcat([colorBufferImage, depthBufferImage])

            #     cv2.imshow(img)
            #     cv2.waitKey(1000//fps)

            # # Close all windows
            # cv2.destroyAllWindows()

    def listener_depth_callback(self, msg):
        self.depthImageSwapReference.set_var1(msg)

    def checkForFalseDetections(self, image:ImageMessage, detection:Detection):
        # TODO
        return detection

    def listener_color_callback(self, msg):
        # Skip the ImageMessage if not enough time has passed since we processed the last one to save CPU - it can't
        # keep up at 30hz even on an i7 13700 -danderson
        if time.time_ns() - self.last_image_process_time_ns < self.image_process_period_ns:
            return
        self.last_image_process_time_ns = time.time_ns()

        self.ros2_node.get_logger().info("Processing ImageMessage #" + str(msg.sequence_number))
        image_np = np.frombuffer(b''.join(msg.data), dtype=np.uint8)
        image = cv2.imdecode(image_np, cv2.COLOR_BGR2RGB)

        x_pixels_width = 720
        y_pixels_height = 200
        x = 1280 - x_pixels_width
        y = 720 - y_pixels_height
        # cv2.rectangle(image, (x, y), (x + x_pixels_width, y + y_pixels_height), (0, 0, 0), -1)
        cv2.rectangle(image, (600, 520), (1280, 720), (0, 0, 0), -1)

        detection:Detection = self.processImage(image)

        if detection is not None and isinstance(detection, Detection):
            detection.cameraPose = Pose(position=msg.position, orientation=msg.orientation)

            # # TODO: use Depth Image
            # detection = checkForFalseDetections(self.depthImageSwapReference.get_var2(), detection)

            self.publish_message(detection)

    def processImage(self, image) -> Optional[Detection]:
        ret = self.detector.run(np.asarray(image), meta_inp=self.meta)

        if len(ret['results']) > 0:
            detection = Detection()

            detection.confidence = ret['results'][0]['score']
            detection.objectType = self.myModelName
            detection.object_id = self.myModelID

            if 'kps_3d_cam' in ret['results'][0]:
                detection.kps_3d = []
                for vertex in ret['results'][0]['kps_3d_cam']: # shape 9x3 (1st row is object centroid location)
                    vertex = vertex/self.scale
                    point = Point(x=vertex[0], y=vertex[1], z=vertex[2])
                    detection.kps_3d.append(point)
                detection.kps_3d = detection.kps_3d[-8:]
            else:
                detection.skipDetaction = True

            if 'kps' in ret['results'][0]:
                detection.kps_2d = []
                detection.kps_2d_array = ret['results'][0]['kps'].reshape(-1, 2)
                bbox = np.array(ret['results'][0]['kps']).reshape(-1, 2)
                for vertex in bbox:
                    point = Point(x=vertex[0], y=vertex[1], z=0.0)
                    detection.kps_2d.append(point)
                detection.kps_2d = detection.kps_2d[-8:]
            else:
                detection.skipDetaction = True

            if 'location' in ret['results'][0]:
                detection.quaternion_xyzw = Rotation.from_quat([ret['results'][0]['quaternion_xyzw'][0],
                                                                ret['results'][0]['quaternion_xyzw'][1],
                                                                ret['results'][0]['quaternion_xyzw'][2],
                                                                ret['results'][0]['quaternion_xyzw'][3]])
                detection.position = np.array([ret['results'][0]['location'][0],
                                               ret['results'][0]['location'][1],
                                               ret['results'][0]['location'][2]])/self.scale
            else:
                detection.skipDetaction = True

            return detection
        else:
            return None

    def publish_message(self, detection:Detection):
        if detection.skipDetaction == False:
            self.detected_object.id = detection.object_id
            detection.sequence_id += 1
            self.detected_object.sequence_id = detection.sequence_id
            self.detected_object.confidence = detection.confidence
            self.detected_object.object_type = detection.objectType
            self.detected_object.bounding_box_vertices = detection.kps_3d
            self.detected_object.bounding_box_2d_vertices = detection.kps_2d
            position = Point(x=detection.position[0], y=detection.position[1], z=detection.position[2])
            quaternion_xyzw = Quaternion(x=detection.quaternion_xyzw.as_quat()[0], y=detection.quaternion_xyzw.as_quat()[1], z=detection.quaternion_xyzw.as_quat()[2], w=detection.quaternion_xyzw.as_quat()[3])
            self.detected_object.pose = Pose(position=position, orientation=quaternion_xyzw)
            self.ros2_node.get_logger().info('Object Detected in the Image!')
        else:
            if (detection.stabilizer<=5):
                detection.sequence_id += 1
                detection.stabilizer += 1
            self.detected_object.sequence_id = detection.sequence_id
            self.ros2_node.get_logger().info('No Object Detected in the Image!')

        self.detected_object.sensor_pose = detection.cameraPose

        self.centerpose_publisher_.publish(self.detected_object)

def main(args=None):
    # Check if models directory exists
    # Download pretrained models from here https://drive.google.com/drive/folders/16HbCnUlCaPcTg4opHP_wQNPsWouUlVZe
    if not os.path.exists('models'):
        print('Could not find models directory')
        return

    experiment = experiment_type.LIVE

    if(experiment==experiment_type.LIVE):
        print("Running CenterPose Live...")
        rclpy.init(args=args)
    
        detectorClass = ZED2CenterposeNode(experiment)
    
        node = detectorClass.ros2_node
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    
    elif (experiment==experiment_type.VIDEO):
        print("Running CenterPose on a Video...")
        detectorClass = ZED2CenterposeNode(experiment)


if __name__ == '__main__':
    main()