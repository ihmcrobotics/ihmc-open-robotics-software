package us.ihmc.atlas.parameters;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.ImmutableTriple;

import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.parameters.AvatarRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotPointCloudParameters;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;

public class AtlasSensorInformation implements HumanoidRobotSensorInformation
{
   public static final long HEAD_MICROSTRAIN_SERIAL_NUMBER = 625476543L;

   private static final String multisense_namespace = "/multisense";
   private static final String realsense_namespace = "/realsense";
   private static final String depth_camera_namespace = "/depthcam";
   private static final String tracking_camera_namespace = "/trackingcam";
   private static final String baseTfName = multisense_namespace + "/head";
   private static final String multisenseHandoffFrame = "head";
   private final ArrayList<ImmutableTriple<String, String, RigidBodyTransform>> staticTranformsForRos = new ArrayList<ImmutableTriple<String, String, RigidBodyTransform>>();

   /**
    * Force Sensor Parameters
    */
   private final String[] forceSensorNames;
   private final SideDependentList<String> feetForceSensorNames = new SideDependentList<String>("l_leg_akx", "r_leg_akx");
   private final SideDependentList<String> handForceSensorNames;

   /**
    * PPS Parameters 
    */
   private static final String MULTISENSE_SL_PPS_TOPIC = multisense_namespace + "/stamped_pps";

   /**
    * Send robot data to ROS
    */
   public static final boolean SEND_ROBOT_DATA_TO_ROS = false;

   /**
    * Camera Parameters
    */
   private final AvatarRobotCameraParameters[] cameraParameters = new AvatarRobotCameraParameters[4];
   public static final int MULTISENSE_SL_LEFT_CAMERA_ID = 0;
   public static final int MULTISENSE_SL_RIGHT_CAMERA_ID = 1;
   public static final int BLACKFLY_LEFT_CAMERA_ID = 2;
   public static final int BLACKFLY_RIGHT_CAMERA_ID = 3;

   private static final String left_camera_name = "stereo_camera_left";
   private static final String left_camera_topic = multisense_namespace + "/left/image_rect_color/compressed";
   private static final String left_info_camera_topic = multisense_namespace + "/left/image_rect_color/camera_info";// left/image_rect_color/camera_info
   private static final String left_frame_name = multisense_namespace + "/left_camera_frame";

   private static final String right_camera_name = "stereo_camera_right";
   private static final String right_camera_topic = multisense_namespace + "/right/image_rect/compressed";
   private static final String right_info_camera_topic = multisense_namespace + "/right/camera_info";
   private static final String right_frame_name = multisense_namespace + "/right_camera_frame";

   private static final String fisheye_pose_source = "utorso";
   private static final String fisheye_left_camera_topic = "/left/camera/image_color/compressed";
   private static final String fisheye_left_camera_info = "/left/camera/camera_info";
   private static final String leftFisheyeCameraName = "l_situational_awareness_camera_sensor_l_situational_awareness_camera";

   private static final String fisheye_right_camera_topic = "/right/camera/image_color/compressed";
   private static final String right_fisheye_camera_name = "r_situational_awareness_camera_sensor_r_situational_awareness_camera";
   private static final String fisheye_right_camera_info = "/right/camera/camera_info";

   public static final String head_imu_acceleration_topic = "/multisense/imu/accelerometer";
   public static final String head_imu_data_topic = "/multisense/imu/imu_data";

   /**
    * Lidar Parameters
    */
   private static final double lidar_spindle_velocity = 2.183;

   private final AvatarRobotLidarParameters[] lidarParameters = new AvatarRobotLidarParameters[1];
   public static final int MULTISENSE_LIDAR_ID = 0;

   private static final String lidarPoseLink = "hokuyo_link";
   private static final String lidarJointName = "hokuyo_joint";
   private static final String lidarEndFrameInSdf = "/head_hokuyo_frame";
   private static final String lidarBaseFrame = multisense_namespace + "/head_root";
   private static final String lidarEndFrame = multisense_namespace + lidarEndFrameInSdf;

   private static final String lidarSensorName = "head_hokuyo_sensor";
   private static final String lidarJointTopic = multisense_namespace + "/joint_states";
   private static final String multisense_laser_topic_string = multisense_namespace + "/lidar_scan";
   private static final String multisense_laser_scan_topic_string = "/singleScanAsCloudWithSource";
   private static final String multisense_laser_topic__as_string = multisense_namespace + "/lidar_points2";
   private static final String multisense_filtered_laser_as_point_cloud_topic_string = multisense_namespace + "/filtered_cloud";
   private static final String multisense_ground_point_cloud_topic_string = multisense_namespace + "/highly_filtered_cloud";
   private static final String bodyIMUSensor = "imu_sensor_at_pelvis_frame";
   private static final String chestIMUSensor = "imu_sensor_chest";
   private static final String[] imuSensorsToUseInStateEstimator = {bodyIMUSensor};

   /**
    * Stereo Parameters
    */

   private final AvatarRobotPointCloudParameters[] pointCloudParameters = new AvatarRobotPointCloudParameters[1];
   public static final int MULTISENSE_STEREO_ID = 0;
   private static final String stereoSensorName = "stereo_camera";
   private static final String stereoColorTopic = multisense_namespace + "/image_points2_color_world";
   private static final String stereoBaseFrame = multisense_namespace + "/head";
   private static final String stereoEndFrame = multisense_namespace + "/left_camera_frame";

   private final boolean isMultisenseHead;
   private final boolean setupROSLocationService;
   private final boolean setupROSParameterSetters;
   private final RobotTarget target;

   public static final double linearVelocityThreshold = 0.2;
   public static final double angularVelocityThreshold = Math.PI / 15;

   /**
    * Realsense Parameters
    */
   public static final String depthCameraTopic = depth_camera_namespace + "/depth/color/points";
   public static final String trackingCameraTopic = tracking_camera_namespace + "/odom/sample";

   private static final double d435DepthOffsetX = 0.058611;
   private static final double d435DepthOffsetZ = 0.01;
   private static final double d435DepthPitchingAngle = 70.0 / 180.0 * Math.PI;
   private static final double d435DepthRollOffset = Math.toRadians(-0.5);
   private static final double d435DepthThickness = 0.0245;

   private static final double trackingOffsetX = 0.0358;
   private static final double trackingOffsetZ = 0.0994;
   private static final double trackingPitchingAngle = 0.0 / 180.0 * Math.PI;
   private static final double trackingThickness = 0.0125;

   private static final double pelvisToMountD435Origin = 0.19;

   public static final RigidBodyTransform transformPelvisToD435DepthCamera = new RigidBodyTransform();
   static
   {
      transformPelvisToD435DepthCamera.appendTranslation(pelvisToMountD435Origin, 0.0, 0.0);
      transformPelvisToD435DepthCamera.appendTranslation(d435DepthOffsetX, 0.0, d435DepthOffsetZ);
      transformPelvisToD435DepthCamera.appendRollRotation(d435DepthRollOffset);
      transformPelvisToD435DepthCamera.appendPitchRotation(d435DepthPitchingAngle);
      transformPelvisToD435DepthCamera.appendTranslation(d435DepthThickness, 0.0, 0.0);

      transformPelvisToD435DepthCamera.appendYawRotation(-Math.PI / 2);
      transformPelvisToD435DepthCamera.appendRollRotation(-Math.PI / 2);
   }

   /**
    * Tracking camera reference frame is, X is forward and Z is toward sky. (Same with Robot coordinate system) 
    */
   public static final RigidBodyTransform transformDepthCameraToTrackingCamera = new RigidBodyTransform();
   static
   {
      transformDepthCameraToTrackingCamera.appendRollRotation(Math.PI / 2);
      transformDepthCameraToTrackingCamera.appendYawRotation(Math.PI / 2);
      transformDepthCameraToTrackingCamera.appendTranslation(-d435DepthThickness, 0.0, 0.0);
      transformDepthCameraToTrackingCamera.appendPitchRotation(-d435DepthPitchingAngle);
      transformDepthCameraToTrackingCamera.appendTranslation(-d435DepthOffsetX, 0.0, -d435DepthOffsetZ);

      transformDepthCameraToTrackingCamera.appendTranslation(trackingOffsetX, 0.0, trackingOffsetZ);
      transformDepthCameraToTrackingCamera.appendPitchRotation(trackingPitchingAngle);
      transformDepthCameraToTrackingCamera.appendRollRotation(-d435DepthRollOffset);
      transformDepthCameraToTrackingCamera.appendTranslation(trackingThickness, 0.0, 0.0);
   }

   public static final RigidBodyTransform transformPelvisToTrackingCamera = new RigidBodyTransform();
   static
   {
      transformPelvisToTrackingCamera.multiply(transformPelvisToD435DepthCamera);
      transformPelvisToTrackingCamera.multiply(transformDepthCameraToTrackingCamera);
   }

   public static final RigidBodyTransform transformTrackingCameraToDepthCamera = new RigidBodyTransform();
   static
   {
      transformTrackingCameraToDepthCamera.multiply(transformDepthCameraToTrackingCamera);
      transformTrackingCameraToDepthCamera.invert();
   }

   private static final RigidBodyTransform transformChestToL515DepthCamera = new RigidBodyTransform();
   static
   {
      // TODO: Move this stuff to a file so it can be tuned and saved
      transformChestToL515DepthCamera.setIdentity();
      transformChestToL515DepthCamera.getTranslation().set(0.275000, 0.052000, 0.140000);
      transformChestToL515DepthCamera.getRotation().setYawPitchRoll(0.010000, 1.151900, 0.045000);
   }

   private static final RigidBodyTransform transformChestToD435DepthCamera = new RigidBodyTransform();
   static
   {
      transformChestToD435DepthCamera.setIdentity();
      transformChestToD435DepthCamera.getTranslation().set(0.300000, 0.020000, 0.580000);
      transformChestToD435DepthCamera.getRotation().setYawPitchRoll(0.0, 0.0, 0.0);
   }

   private static final RigidBodyTransform transformChestToOuster = new RigidBodyTransform();
   static
   {
      transformChestToOuster.setIdentity();
      transformChestToOuster.getTranslation().set(0.265000, -0.0200, 0.720000);
      transformChestToOuster.getRotation().setYawPitchRoll(0.00000, 0.52400, 0.000000);
   }

   private static final RigidBodyTransform transformChestToZED2 = new RigidBodyTransform();
   static
   {
      Point3D chestToSensor = new Point3D(0.275, 0.052, 0.4);
      transformChestToZED2.appendTranslation(chestToSensor);
      double pitch = Math.toRadians(90.0 - 24.0);
      transformChestToZED2.appendOrientation(new YawPitchRoll(0.01, pitch, -0.045));
   }

   private static final RigidBodyTransform transformChestToRightBlackfly = new RigidBodyTransform();
   static
   {
      transformChestToRightBlackfly.setIdentity();
      transformChestToRightBlackfly.getTranslation().set(transformChestToOuster.getTranslation()); // measured relative to Ouster for convenience
      transformChestToRightBlackfly.getTranslation().add(0.02, -0.12, -0.15);
      transformChestToRightBlackfly.getRotation().setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(35.0), Math.toRadians(-5.0));
   }

   public AtlasSensorInformation(AtlasRobotVersion atlasRobotVersion, RobotTarget target)
   {
      this.target = target;

      if (atlasRobotVersion != AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_FOREARMS)
      {
         forceSensorNames = new String[] {"l_leg_akx", "r_leg_akx", "l_arm_wry2", "r_arm_wry2"};
         handForceSensorNames = new SideDependentList<String>("l_arm_wry2", "r_arm_wry2");
      }
      else
      {
         forceSensorNames = new String[] {"l_leg_akx", "r_leg_akx"};
         handForceSensorNames = null;
      }

      if (target == RobotTarget.REAL_ROBOT)
      {
         cameraParameters[MULTISENSE_SL_LEFT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.LEFT, left_camera_name, left_camera_topic,
                                                                                          left_info_camera_topic, multisenseHandoffFrame, baseTfName,
                                                                                          left_frame_name, MULTISENSE_SL_LEFT_CAMERA_ID);
         cameraParameters[MULTISENSE_SL_RIGHT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.RIGHT, right_camera_name, right_camera_topic,
                                                                                           right_info_camera_topic, multisenseHandoffFrame, baseTfName,
                                                                                           right_frame_name, MULTISENSE_SL_RIGHT_CAMERA_ID);
         lidarParameters[MULTISENSE_LIDAR_ID] = new AvatarRobotLidarParameters(true, lidarSensorName, multisense_laser_scan_topic_string,
                                                                               multisense_laser_scan_topic_string, lidarJointName, lidarJointTopic,
                                                                               multisenseHandoffFrame, lidarBaseFrame, lidarEndFrame, lidar_spindle_velocity,
                                                                               MULTISENSE_LIDAR_ID);
         pointCloudParameters[MULTISENSE_STEREO_ID] = new AvatarRobotPointCloudParameters(stereoSensorName, stereoColorTopic, multisenseHandoffFrame,
                                                                                          stereoBaseFrame, stereoEndFrame, MULTISENSE_STEREO_ID);
      }
      else if (target == RobotTarget.HEAD_ON_A_STICK)
      {
         cameraParameters[MULTISENSE_SL_LEFT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.LEFT, left_camera_name, left_camera_topic,
                                                                                          left_info_camera_topic, multisenseHandoffFrame, baseTfName,
                                                                                          left_frame_name, MULTISENSE_SL_LEFT_CAMERA_ID);
         cameraParameters[MULTISENSE_SL_RIGHT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.RIGHT, right_camera_name, right_camera_topic,
                                                                                           right_info_camera_topic, multisenseHandoffFrame, baseTfName,
                                                                                           right_frame_name, MULTISENSE_SL_RIGHT_CAMERA_ID);
         lidarParameters[MULTISENSE_LIDAR_ID] = new AvatarRobotLidarParameters(true, lidarSensorName, multisense_filtered_laser_as_point_cloud_topic_string,
                                                                               multisense_ground_point_cloud_topic_string, lidarJointName, lidarJointTopic,
                                                                               multisenseHandoffFrame, lidarBaseFrame, lidarEndFrame, lidar_spindle_velocity,
                                                                               MULTISENSE_LIDAR_ID);
         pointCloudParameters[MULTISENSE_STEREO_ID] = new AvatarRobotPointCloudParameters(stereoSensorName, stereoColorTopic, multisenseHandoffFrame,
                                                                                          stereoBaseFrame, stereoEndFrame, MULTISENSE_STEREO_ID);
      }
      else if (target == RobotTarget.GAZEBO)
      {
         String baseTfName = "head";
         String left_frame_name = "left_camera_frame";
         String right_frame_name = "right_camera_frame";
         String lidarBaseFrame = "head";
         String lidarEndFrame = "head_hokuyo_frame";

         cameraParameters[MULTISENSE_SL_LEFT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.LEFT, left_camera_name, left_camera_topic,
                                                                                          left_info_camera_topic, multisenseHandoffFrame, baseTfName,
                                                                                          left_frame_name, MULTISENSE_SL_LEFT_CAMERA_ID);
         cameraParameters[MULTISENSE_SL_RIGHT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.RIGHT, right_camera_name, right_camera_topic,
                                                                                           right_info_camera_topic, multisenseHandoffFrame, baseTfName,
                                                                                           right_frame_name, MULTISENSE_SL_RIGHT_CAMERA_ID);
         lidarParameters[MULTISENSE_LIDAR_ID] = new AvatarRobotLidarParameters(true, lidarSensorName, multisense_laser_topic_string,
                                                                               multisense_laser_topic_string, lidarJointName, lidarJointTopic,
                                                                               multisenseHandoffFrame, lidarBaseFrame, lidarEndFrame, lidar_spindle_velocity,
                                                                               MULTISENSE_LIDAR_ID);
         pointCloudParameters[MULTISENSE_STEREO_ID] = new AvatarRobotPointCloudParameters(stereoSensorName, stereoColorTopic, multisenseHandoffFrame,
                                                                                          stereoBaseFrame, stereoEndFrame, MULTISENSE_STEREO_ID);
      }
      else
      {
         cameraParameters[MULTISENSE_SL_LEFT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.LEFT, left_camera_name, left_camera_topic,
                                                                                          multisenseHandoffFrame, left_info_camera_topic,
                                                                                          MULTISENSE_SL_LEFT_CAMERA_ID);
         cameraParameters[MULTISENSE_SL_RIGHT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.RIGHT, right_camera_name, right_camera_topic,
                                                                                           multisenseHandoffFrame, right_info_camera_topic,
                                                                                           MULTISENSE_SL_RIGHT_CAMERA_ID);
         lidarParameters[MULTISENSE_LIDAR_ID] = new AvatarRobotLidarParameters(false, lidarSensorName, multisense_laser_topic_string,
                                                                               multisense_laser_topic_string, lidarJointName, lidarJointTopic, lidarPoseLink,
                                                                               multisenseHandoffFrame, lidarEndFrameInSdf, lidar_spindle_velocity,
                                                                               MULTISENSE_LIDAR_ID);
         pointCloudParameters[MULTISENSE_STEREO_ID] = new AvatarRobotPointCloudParameters(stereoSensorName, stereoColorTopic, multisenseHandoffFrame,
                                                                                          MULTISENSE_STEREO_ID);
      }

      cameraParameters[BLACKFLY_LEFT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.LEFT, leftFisheyeCameraName, fisheye_left_camera_topic,
                                                                                  fisheye_pose_source, fisheye_left_camera_info, BLACKFLY_LEFT_CAMERA_ID);
      cameraParameters[BLACKFLY_RIGHT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.RIGHT, right_fisheye_camera_name, fisheye_right_camera_topic,
                                                                                   fisheye_pose_source, fisheye_right_camera_info, BLACKFLY_RIGHT_CAMERA_ID);

      setupROSLocationService = false; //target == RobotTarget.REAL_ROBOT || (target == RobotTarget.SCS && SEND_ROBOT_DATA_TO_ROS);
      setupROSParameterSetters = false; //target == RobotTarget.REAL_ROBOT;
      isMultisenseHead = false; //target == RobotTarget.REAL_ROBOT;

      setupStaticTransformsForRos();
   }

   private void setupStaticTransformsForRos()
   {
      ImmutableTriple<String, String, RigidBodyTransform> headToHeadRootStaticTransform = new ImmutableTriple<String, String, RigidBodyTransform>("head",
                                                                                                                                                  "multisense/head_root",
                                                                                                                                                  new RigidBodyTransform());
      staticTranformsForRos.add(headToHeadRootStaticTransform);
   }

   @Override
   public AvatarRobotLidarParameters[] getLidarParameters()
   {
      return lidarParameters;
   }

   @Override
   public AvatarRobotLidarParameters getLidarParameters(int sensorId)
   {
      return lidarParameters[sensorId];
   }

   @Override
   public String[] getIMUSensorsToUseInStateEstimator()
   {
      return imuSensorsToUseInStateEstimator;
   }

   @Override
   public String getPrimaryBodyImu()
   {
      return bodyIMUSensor;
   }

   public String getChestImu()
   {
      return chestIMUSensor;
   }

   @Override
   public String[] getForceSensorNames()
   {
      return forceSensorNames;
   }

   @Override
   public SideDependentList<String> getFeetForceSensorNames()
   {
      return feetForceSensorNames;
   }

   @Override
   public SideDependentList<String> getWristForceSensorNames()
   {
      return handForceSensorNames;
   }

   @Override
   public AvatarRobotCameraParameters[] getCameraParameters()
   {
      return cameraParameters;
   }

   @Override
   public AvatarRobotCameraParameters getCameraParameters(int sensorId)
   {
      return cameraParameters[sensorId];
   }

   public String getCameraStringBase()
   {
      return multisense_namespace;
   }

   public String getPPSRosTopic()
   {
      return MULTISENSE_SL_PPS_TOPIC;
   }

   @Override
   public AvatarRobotPointCloudParameters[] getPointCloudParameters()
   {
      return pointCloudParameters;
   }

   @Override
   public AvatarRobotPointCloudParameters getPointCloudParameters(int sensorId)
   {
      return pointCloudParameters[sensorId];
   }

   @Override
   public boolean setupROSLocationService()
   {
      return setupROSLocationService;
   }

   @Override
   public boolean setupROSParameterSetters()
   {
      return setupROSParameterSetters;
   }

   @Override
   public boolean isMultisenseHead()
   {
      return isMultisenseHead;
   }

   @Override
   public ArrayList<ImmutableTriple<String, String, RigidBodyTransform>> getStaticTransformsForRos()
   {
      return staticTranformsForRos;
   }

   @Override
   public RigidBodyTransform getSteppingCameraTransform()
   {
      return transformChestToL515DepthCamera;
   }

   @Override
   public ReferenceFrame getSteppingCameraParentFrame(CommonHumanoidReferenceFrames referenceFrames)
   {
      return referenceFrames.getChestFrame();
   }

   @Override
   public RigidBodyTransform getObjectDetectionCameraTransform()
   {
      return transformChestToRightBlackfly;
   }

   @Override
   public String getHeadCameraName()
   {
      return "stereo_camera_left";
   }

   @Override
   public RigidBodyTransform getExperimentalCameraTransform()
   {
      return transformChestToZED2;
   }

   @Override
   public RigidBodyTransform getOusterLidarTransform()
   {
      return transformChestToOuster;
   }
}
