package us.ihmc.atlas.parameters;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotCameraParamaters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.graphics3DAdapter.camera.VideoSettingsFactory;
import us.ihmc.graphics3DAdapter.camera.VideoSettingsH264LowLatency;
import us.ihmc.robotSide.SideDependentList;

public class AtlasSensorInformation implements DRCRobotSensorInformation
{
   public static final String[] forceSensorNames = { "l_leg_akx", "r_leg_akx", "l_arm_wrx", "r_arm_wrx" };
   public static final SideDependentList<String> feetForceSensorNames = new SideDependentList<String>("l_leg_akx", "r_leg_akx");
   public static final SideDependentList<String> handForceSensorNames = new SideDependentList<String>("l_arm_wrx", "r_arm_wrx");
   private final DRCRobotCameraParamaters[] cameraParamaters = new DRCRobotCameraParamaters[4];
   
   private static final int multisense_sl_left_camera_id = 0;
   private static final int multisense_sl_right_camera_id = 1;
   private static final int blackfly_left_camera_id = 2;
   private static final int blackfly_right_camera_id = 3;
   private static final int primaryCameraId = multisense_sl_left_camera_id;
   
   private static final String multisense_topic_base = "/multisense";
   private static final String baseTfName = multisense_topic_base + "/head";
   private static final String left_camera_name = "stereo_camera_left";
   private static final String left_camera_topic = multisense_topic_base + "/left/image_rect_color/compressed";
   private static final String left_info_camera_topic = multisense_topic_base +"/left/camera_info";
   private static final String left_frame_name = multisense_topic_base+"/left_camera_frame";
   
   private static final String right_camera_name = "stereo_camera_right";
   private static final String right_camera_topic = multisense_topic_base + "/right/image_rect/compressed";
   private static final String right_info_camera_topic = multisense_topic_base +"/right/camera_info";
   private static final String right_frame_name = multisense_topic_base+"/right_camera_frame";
   
   private static final String fisheye_left_camera_topic = "/blackfly/camera/left/compressed";
   private static final String leftFisheyeCameraName = "l_situational_awareness_camera";
                        
   private static final String fisheye_right_camera_topic = "/blackfly/camera/right/compressed";
   private static final String rightFisheyeCameraName = "r_situational_awareness_camera";
   
   /*
    * multisense driver publishes tf from /multisense/head_root -> /multisense/head_hokuyo_frame
    * This has to be acquired runtime (not hardcoded here) from tf since it includes lidar-camera pose calibration information
    * The new model does not contain hokuyo_joint anymore (which may still be used by gazebo)
    * change since multisense 3.0
    * -Tingfan Wu
    */
   private final String lidarJointName; //this has to match LidarDataReceiver::LIDAR_HEAD_FRAME; gazebo should use: "hokuyo_joint"; 
   private static final String lidarSensorName = "head_hokuyo_sensor";
   private static final String lidarJointTopic = multisense_topic_base + "joint_states";
   private static final String multisense_laser_topic_string = multisense_topic_base+"/lidar_scan";
   private static final String bodyIMUSensor = "pelvis_imu_sensor";
   private static final String[] imuSensorsToUse = { bodyIMUSensor };
   
   
   public AtlasSensorInformation(boolean runningOnRealRobot)
   {
      VideoSettingsH264LowLatency videoSetting = VideoSettingsFactory.get32kBitSettingsSquare();
      if(runningOnRealRobot)
      {
         videoSetting = VideoSettingsFactory.get32kBitSettingsWide();
      }
      
      lidarJointName = runningOnRealRobot ? "head" : "hokuyo_joint";

      cameraParamaters[0] = new DRCRobotCameraParamaters(left_camera_name, left_camera_topic, left_info_camera_topic, left_frame_name, baseTfName, videoSetting, multisense_sl_left_camera_id);
      cameraParamaters[1] = new DRCRobotCameraParamaters(right_camera_name, right_camera_topic, right_info_camera_topic, right_frame_name, baseTfName, videoSetting, multisense_sl_right_camera_id);
      
      cameraParamaters[2] = new DRCRobotCameraParamaters(leftFisheyeCameraName, fisheye_left_camera_topic, leftFisheyeCameraName, videoSetting, blackfly_left_camera_id);
      cameraParamaters[3] = new DRCRobotCameraParamaters(rightFisheyeCameraName, fisheye_right_camera_topic, rightFisheyeCameraName, videoSetting, blackfly_right_camera_id);
   }
   
   @Override
   public String getLidarJointName()
   {
      return lidarJointName;
   }

   @Override
   public String getLidarSensorName()
   {
      return lidarSensorName;
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

   @Override
   public String getLidarScanTopic()
   {
      return multisense_laser_topic_string;
   }

   public String getCameraStringBase()
   {
      return multisense_topic_base;
   }

   @Override
   public String getLidarJointTopic()
   {
      return lidarJointTopic;
   }
}
