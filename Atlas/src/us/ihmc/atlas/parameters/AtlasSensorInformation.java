package us.ihmc.atlas.parameters;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotCameraParamaters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotLidarParamaters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPointCloudParamaters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.graphics3DAdapter.camera.VideoSettingsFactory;
import us.ihmc.graphics3DAdapter.camera.VideoSettingsH264LowLatency;
import us.ihmc.robotSide.SideDependentList;

public class AtlasSensorInformation implements DRCRobotSensorInformation
{
   private static final String multisense_namespace = "/multisense";
   private static final String baseTfName = multisense_namespace + "/head";

   /**
    * Force Sensor Parameters
    */
   public static final String[] forceSensorNames = { "l_leg_akx", "r_leg_akx", "l_arm_wrx", "r_arm_wrx" };
   public static final SideDependentList<String> feetForceSensorNames = new SideDependentList<String>("l_leg_akx", "r_leg_akx");
   public static final SideDependentList<String> handForceSensorNames = new SideDependentList<String>("l_arm_wrx", "r_arm_wrx");
   
   /**
    * PPS Parameters
    */
   private static final int PPS_PROVIDER_PORT = 5050;
   private static final String MULTISENSE_SL_PPS_TOPIC = "/multisense/pps";
   
   /**
    * Camera Parameters
    */
   private final DRCRobotCameraParamaters[] cameraParamaters = new DRCRobotCameraParamaters[4];
   private static final int multisense_sl_left_camera_id = 0;
   private static final int multisense_sl_right_camera_id = 1;
   private static final int blackfly_left_camera_id = 2;
   private static final int blackfly_right_camera_id = 3;
   private static final int primaryCameraId = multisense_sl_left_camera_id;
   
   private static final String left_camera_name = "stereo_camera_left";
   private static final String left_camera_topic = multisense_namespace + "/left/image_rect_color/compressed";
   private static final String left_info_camera_topic = multisense_namespace +"/left/camera_info";
   private static final String left_frame_name = multisense_namespace + "/left_camera_optical_frame";
   
   private static final String right_camera_name = "stereo_camera_right";
   private static final String right_camera_topic = multisense_namespace + "/right/image_rect/compressed";
   private static final String right_info_camera_topic = multisense_namespace +"/right/camera_info";
   private static final String right_frame_name = multisense_namespace + "/right_camera_optical_frame";
   
   private static final String fisheye_left_camera_topic = "/blackfly/camera/left/compressed";
   private static final String leftFisheyeCameraName = "l_situational_awareness_camera";
                        
   private static final String fisheye_right_camera_topic = "/blackfly/camera/right/compressed";
   private static final String right_fisheye_camera_name = "r_situational_awareness_camera";
   
   /**
    * Lidar Parameters
    */
   private static final double lidar_spindle_velocity = 5.1;
   
   private final DRCRobotLidarParamaters[] lidarParamaters = new DRCRobotLidarParamaters[1];
   private final int multiSenseLidarId = 0;
   private final String lidarJointName; //this has to match LidarDataReceiver::LIDAR_HEAD_FRAME; gazebo should use: "hokuyo_joint"; 
   private final String lidarBaseFrame = multisense_namespace + "/head";
   private final String lidarEndFrame = multisense_namespace + "/head_hokuyo_frame";
   
   private static final String lidarSensorName = "head_hokuyo_sensor";
   private static final String lidarJointTopic = multisense_namespace + "joint_states";
   private static final String multisense_laser_topic_string = multisense_namespace+"/lidar_scan";
   private static final String bodyIMUSensor = "pelvis_imu_sensor";
   private static final String[] imuSensorsToUse = { bodyIMUSensor };
   
   /**
    * Stereo Parameters
    */
   private final DRCRobotPointCloudParamaters[] pointCloudParamaters = new DRCRobotPointCloudParamaters[1];
   private final int multiSenseStereoId = 0;
   private static final String stereoSensorName = "stereo_camera";
   private static final String stereoColorTopic = multisense_namespace + "image_points2_color";
   private final String stereoBaseFrame = multisense_namespace + "/head";
   private final String stereoEndFrame = multisense_namespace + "/left_camera_optical_frame";
   
   public AtlasSensorInformation(boolean runningOnRealRobot)
   {
      VideoSettingsH264LowLatency videoSetting = VideoSettingsFactory.get32kBitSettingsWide();
      cameraParamaters[0] = new DRCRobotCameraParamaters(left_camera_name, left_camera_topic, left_info_camera_topic, left_frame_name, baseTfName, videoSetting, multisense_sl_left_camera_id);
      cameraParamaters[1] = new DRCRobotCameraParamaters(right_camera_name, right_camera_topic, right_info_camera_topic, right_frame_name, baseTfName, videoSetting, multisense_sl_right_camera_id);
      cameraParamaters[2] = new DRCRobotCameraParamaters(leftFisheyeCameraName, fisheye_left_camera_topic, leftFisheyeCameraName, videoSetting, blackfly_left_camera_id);
      cameraParamaters[3] = new DRCRobotCameraParamaters(right_fisheye_camera_name, fisheye_right_camera_topic, right_fisheye_camera_name, videoSetting, blackfly_right_camera_id);
      lidarJointName = runningOnRealRobot ? "head" : "hokuyo_joint";
      lidarParamaters[0] = new DRCRobotLidarParamaters(lidarSensorName, multisense_laser_topic_string, lidarJointName, lidarJointTopic, lidarBaseFrame, lidarEndFrame, lidar_spindle_velocity);
      
      pointCloudParamaters[0] = new DRCRobotPointCloudParamaters(stereoSensorName, stereoColorTopic, stereoBaseFrame, stereoEndFrame);
   }
   
   @Override
   public DRCRobotLidarParamaters[] getLidarParamaters()
   {
      return lidarParamaters;
   }
   
   @Override
   public DRCRobotLidarParamaters getPrimaryLidarParameters()
   {
      return lidarParamaters[multiSenseLidarId];
   }

   @Override
   public String[] getIMUSensorsToUse()
   {
      return imuSensorsToUse;
   }
   
   @Override
   public String getPrimaryBodyImu()
   {
      return bodyIMUSensor;
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
   public DRCRobotCameraParamaters[] getCameraParamaters()
   {
      return cameraParamaters;
   }

   @Override
   public DRCRobotCameraParamaters getPrimaryCameraParamaters()
   {
      return cameraParamaters[primaryCameraId];
   }

   public String getCameraStringBase()
   {
      return multisense_namespace;
   }

   public int getPPSProviderPort()
   {
      return PPS_PROVIDER_PORT;
   }

   public String getPPSRosTopic()
   {
      return MULTISENSE_SL_PPS_TOPIC;
   }

   @Override
   public DRCRobotPointCloudParamaters[] getPointCloudParamaters()
   {
      return pointCloudParamaters;
   }

   @Override
   public DRCRobotPointCloudParamaters getPrimaryPointCloudParameters()
   {
      return pointCloudParamaters[multiSenseStereoId];
   }
}
