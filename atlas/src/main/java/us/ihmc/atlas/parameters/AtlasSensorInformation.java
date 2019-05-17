package us.ihmc.atlas.parameters;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.ImmutableTriple;

import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.*;
import us.ihmc.sensorProcessing.parameters.AvatarRobotCameraParameters;

public class AtlasSensorInformation implements AvatarHumanoidRobotSensorInformation
{
   private static final String multisense_namespace = "/multisense";
   private static final String baseTfName = multisense_namespace + "/head";
   private static final String multisenseHandoffFrame = "head";
   private final ArrayList<ImmutableTriple<String, String, RigidBodyTransform>> staticTranformsForRos = new ArrayList<ImmutableTriple<String,String,RigidBodyTransform>>();

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
   private static final String left_info_camera_topic = multisense_namespace +"/left/image_rect_color/camera_info";//left/image_rect_color/camera_info
   private static final String left_frame_name = multisense_namespace + "/left_camera_frame";

   private static final String right_camera_name = "stereo_camera_right";
   private static final String right_camera_topic = multisense_namespace + "/right/image_rect/compressed";
   private static final String right_info_camera_topic = multisense_namespace +"/right/camera_info";
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
   private static final String multisense_laser_topic_string = multisense_namespace+"/lidar_scan";
   private static final String multisense_laser_scan_topic_string = "/singleScanAsCloudWithSource";
   private static final String multisense_laser_topic__as_string = multisense_namespace+"/lidar_points2";
   private static final String multisense_filtered_laser_as_point_cloud_topic_string = multisense_namespace+"/filtered_cloud";
   private static final String multisense_ground_point_cloud_topic_string = multisense_namespace+"/highly_filtered_cloud";
   private static final String bodyIMUSensor = "pelvis_imu_sensor_at_pelvis_frame";
   private static final String chestIMUSensor = "utorso_imu_sensor_chest";
   private static final String[] imuSensorsToUseInStateEstimator = { bodyIMUSensor };

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

   public AtlasSensorInformation(AtlasRobotVersion atlasRobotVersion, RobotTarget target)
   {
	   this.target = target;

	   if (atlasRobotVersion != AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_FOREARMS)
	   {
	      forceSensorNames = new String[]{ "l_leg_akx", "r_leg_akx", "l_arm_wry2", "r_arm_wry2" };
	      handForceSensorNames = new SideDependentList<String>("l_arm_wry2", "r_arm_wry2");
	   }
	   else
	   {
	      forceSensorNames = new String[]{ "l_leg_akx", "r_leg_akx" };
	      handForceSensorNames = null;
	   }

      if(target == RobotTarget.REAL_ROBOT)
      {
         cameraParameters[MULTISENSE_SL_LEFT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.LEFT, left_camera_name, left_camera_topic, left_info_camera_topic, multisenseHandoffFrame, baseTfName, left_frame_name, MULTISENSE_SL_LEFT_CAMERA_ID);
         cameraParameters[MULTISENSE_SL_RIGHT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.RIGHT, right_camera_name, right_camera_topic, right_info_camera_topic, multisenseHandoffFrame, baseTfName, right_frame_name, MULTISENSE_SL_RIGHT_CAMERA_ID);
         lidarParameters[MULTISENSE_LIDAR_ID] = new AvatarRobotLidarParameters(true, lidarSensorName, multisense_laser_scan_topic_string,
                                                                               multisense_laser_scan_topic_string, lidarJointName, lidarJointTopic, multisenseHandoffFrame, lidarBaseFrame, lidarEndFrame, lidar_spindle_velocity, MULTISENSE_LIDAR_ID);
         pointCloudParameters[MULTISENSE_STEREO_ID] = new AvatarRobotPointCloudParameters(stereoSensorName, stereoColorTopic, multisenseHandoffFrame, stereoBaseFrame, stereoEndFrame, MULTISENSE_STEREO_ID);
      }
      else if(target == RobotTarget.HEAD_ON_A_STICK)
      {
         cameraParameters[MULTISENSE_SL_LEFT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.LEFT, left_camera_name, left_camera_topic, left_info_camera_topic, multisenseHandoffFrame, baseTfName, left_frame_name, MULTISENSE_SL_LEFT_CAMERA_ID);
         cameraParameters[MULTISENSE_SL_RIGHT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.RIGHT, right_camera_name, right_camera_topic, right_info_camera_topic, multisenseHandoffFrame, baseTfName, right_frame_name, MULTISENSE_SL_RIGHT_CAMERA_ID);
         lidarParameters[MULTISENSE_LIDAR_ID] = new AvatarRobotLidarParameters(true, lidarSensorName, multisense_filtered_laser_as_point_cloud_topic_string,
                                                                               multisense_ground_point_cloud_topic_string, lidarJointName, lidarJointTopic, multisenseHandoffFrame, lidarBaseFrame, lidarEndFrame, lidar_spindle_velocity, MULTISENSE_LIDAR_ID);
         pointCloudParameters[MULTISENSE_STEREO_ID] = new AvatarRobotPointCloudParameters(stereoSensorName, stereoColorTopic, multisenseHandoffFrame, stereoBaseFrame, stereoEndFrame, MULTISENSE_STEREO_ID);
      }
      else if (target == RobotTarget.GAZEBO)
      {
         String baseTfName = "head";
         String left_frame_name = "left_camera_frame";
         String right_frame_name = "right_camera_frame";
         String lidarBaseFrame = "head";
         String lidarEndFrame = "head_hokuyo_frame";


         cameraParameters[MULTISENSE_SL_LEFT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.LEFT, left_camera_name, left_camera_topic, left_info_camera_topic,
                                                                                          multisenseHandoffFrame, baseTfName, left_frame_name, MULTISENSE_SL_LEFT_CAMERA_ID);
         cameraParameters[MULTISENSE_SL_RIGHT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.RIGHT, right_camera_name, right_camera_topic, right_info_camera_topic,
                                                                                           multisenseHandoffFrame, baseTfName, right_frame_name, MULTISENSE_SL_RIGHT_CAMERA_ID);
         lidarParameters[MULTISENSE_LIDAR_ID] = new AvatarRobotLidarParameters(true, lidarSensorName, multisense_laser_topic_string, multisense_laser_topic_string,
                                                                               lidarJointName, lidarJointTopic, multisenseHandoffFrame, lidarBaseFrame, lidarEndFrame, lidar_spindle_velocity, MULTISENSE_LIDAR_ID);
         pointCloudParameters[MULTISENSE_STEREO_ID] = new AvatarRobotPointCloudParameters(stereoSensorName, stereoColorTopic, multisenseHandoffFrame,
                                                                                          stereoBaseFrame, stereoEndFrame, MULTISENSE_STEREO_ID);
      }
      else
      {
         cameraParameters[MULTISENSE_SL_LEFT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.LEFT, left_camera_name, left_camera_topic, multisenseHandoffFrame, left_info_camera_topic, MULTISENSE_SL_LEFT_CAMERA_ID);
         cameraParameters[MULTISENSE_SL_RIGHT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.RIGHT, right_camera_name, right_camera_topic, multisenseHandoffFrame, right_info_camera_topic, MULTISENSE_SL_RIGHT_CAMERA_ID);
         lidarParameters[MULTISENSE_LIDAR_ID] = new AvatarRobotLidarParameters(false, lidarSensorName, multisense_laser_topic_string, multisense_laser_topic_string,
                                                                               lidarJointName, lidarJointTopic, lidarPoseLink, multisenseHandoffFrame, lidarEndFrameInSdf, lidar_spindle_velocity, MULTISENSE_LIDAR_ID);
         pointCloudParameters[MULTISENSE_STEREO_ID] = new AvatarRobotPointCloudParameters(stereoSensorName, stereoColorTopic, multisenseHandoffFrame, MULTISENSE_STEREO_ID);
      }

      cameraParameters[BLACKFLY_LEFT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.LEFT, leftFisheyeCameraName, fisheye_left_camera_topic, fisheye_pose_source, fisheye_left_camera_info, BLACKFLY_LEFT_CAMERA_ID);
      cameraParameters[BLACKFLY_RIGHT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.RIGHT, right_fisheye_camera_name, fisheye_right_camera_topic, fisheye_pose_source, fisheye_right_camera_info, BLACKFLY_RIGHT_CAMERA_ID);

      setupROSLocationService = target == RobotTarget.REAL_ROBOT || (target == RobotTarget.SCS && SEND_ROBOT_DATA_TO_ROS);
      setupROSParameterSetters = target == RobotTarget.REAL_ROBOT;
      isMultisenseHead = target == RobotTarget.REAL_ROBOT;

      setupStaticTransformsForRos();
	}

	private void setupStaticTransformsForRos()
   {
	   ImmutableTriple<String, String, RigidBodyTransform> headToHeadRootStaticTransform = new ImmutableTriple<String, String, RigidBodyTransform>("head", "multisense/head_root", new RigidBodyTransform());
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

   private void sensorFramesToTrack(AvatarRobotSensorParameters[] sensorParams, ArrayList<String> holder)
   {
      for(int i = 0; i < sensorParams.length; i++)
      {
         if(sensorParams[i].getPoseFrameForSdf() != null)
         {
            holder.add(sensorParams[i].getPoseFrameForSdf());
         }
      }
   }

   @Override
   public String[] getSensorFramesToTrack()
   {
      ArrayList<String> sensorFramesToTrack = new ArrayList<String>();
      sensorFramesToTrack(cameraParameters, sensorFramesToTrack);
      sensorFramesToTrack(lidarParameters, sensorFramesToTrack);
      sensorFramesToTrack(pointCloudParameters, sensorFramesToTrack);
      String[] sensorFramesToTrackAsPrimitive = new String[sensorFramesToTrack.size()];
      sensorFramesToTrack.toArray(sensorFramesToTrackAsPrimitive);
      return sensorFramesToTrackAsPrimitive;
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
   public SideDependentList<String> getFeetContactSensorNames()
   {
      return new SideDependentList<String>();
   }

   @Override
   public ArrayList<ImmutableTriple<String, String, RigidBodyTransform>> getStaticTransformsForRos()
   {
      return staticTranformsForRos;
   }

}
