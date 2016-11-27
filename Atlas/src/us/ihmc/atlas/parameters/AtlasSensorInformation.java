package us.ihmc.atlas.parameters;

import java.util.ArrayList;
import java.util.EnumMap;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.tuple.ImmutableTriple;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotPointCloudParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorParameters;

public class AtlasSensorInformation implements DRCRobotSensorInformation
{
   private static final String multisense_namespace = "/multisense";
   private static final String baseTfName = multisense_namespace + "/head";
   private static final String multisenseHandoffFrame = "head";
   private final ArrayList<ImmutableTriple<String, String, RigidBodyTransform>> staticTranformsForRos = new ArrayList<ImmutableTriple<String,String,RigidBodyTransform>>();

   /**
    * Force Sensor Parameters
    */
   public static final String[] forceSensorNames = { "l_leg_akx", "r_leg_akx", "l_arm_wry2", "r_arm_wry2" };
   public static final SideDependentList<String> feetForceSensorNames = new SideDependentList<String>("l_leg_akx", "r_leg_akx");
   public static final SideDependentList<String> handForceSensorNames = new SideDependentList<String>("l_arm_wry2", "r_arm_wry2");

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
   private final DRCRobotCameraParameters[] cameraParameters = new DRCRobotCameraParameters[4];
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

   private final DRCRobotLidarParameters[] lidarParameters = new DRCRobotLidarParameters[1];
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
   private static EnumMap<DRCRobotModel.RobotTarget, ReferenceFrame> headIMUFramesWhenLevel=new EnumMap<>(DRCRobotModel.RobotTarget.class);

   /**
    * Stereo Parameters
    */

   private final DRCRobotPointCloudParameters[] pointCloudParameters = new DRCRobotPointCloudParameters[1];
   public static final int MULTISENSE_STEREO_ID = 0;
   private static final String stereoSensorName = "stereo_camera";
   private static final String stereoColorTopic = multisense_namespace + "image_points2_color";
   private static final String stereoBaseFrame = multisense_namespace + "/head";
   private static final String stereoEndFrame = multisense_namespace + "/left_camera_frame";

   private final boolean isMultisenseHead;
   private final boolean setupROSLocationService;
   private final boolean setupROSParameterSetters;
   private final DRCRobotModel.RobotTarget target;

   public AtlasSensorInformation(DRCRobotModel.RobotTarget target)
   {
	   this.target = target;
      if(target == DRCRobotModel.RobotTarget.REAL_ROBOT)
      {
         cameraParameters[MULTISENSE_SL_LEFT_CAMERA_ID] = new DRCRobotCameraParameters(RobotSide.LEFT, left_camera_name, left_camera_topic, left_info_camera_topic, multisenseHandoffFrame, baseTfName, left_frame_name, MULTISENSE_SL_LEFT_CAMERA_ID);
         cameraParameters[MULTISENSE_SL_RIGHT_CAMERA_ID] = new DRCRobotCameraParameters(RobotSide.RIGHT, right_camera_name, right_camera_topic, right_info_camera_topic, multisenseHandoffFrame, baseTfName, right_frame_name, MULTISENSE_SL_RIGHT_CAMERA_ID);
         lidarParameters[MULTISENSE_LIDAR_ID] = new DRCRobotLidarParameters(true, lidarSensorName, multisense_laser_scan_topic_string,
               multisense_laser_scan_topic_string, lidarJointName, lidarJointTopic, multisenseHandoffFrame, lidarBaseFrame, lidarEndFrame, lidar_spindle_velocity, MULTISENSE_LIDAR_ID);
         pointCloudParameters[MULTISENSE_STEREO_ID] = new DRCRobotPointCloudParameters(stereoSensorName, stereoColorTopic, multisenseHandoffFrame, stereoBaseFrame, stereoEndFrame, MULTISENSE_STEREO_ID);
      }
      else if(target == DRCRobotModel.RobotTarget.HEAD_ON_A_STICK)
      {
         cameraParameters[MULTISENSE_SL_LEFT_CAMERA_ID] = new DRCRobotCameraParameters(RobotSide.LEFT, left_camera_name, left_camera_topic, left_info_camera_topic, multisenseHandoffFrame, baseTfName, left_frame_name, MULTISENSE_SL_LEFT_CAMERA_ID);
         cameraParameters[MULTISENSE_SL_RIGHT_CAMERA_ID] = new DRCRobotCameraParameters(RobotSide.RIGHT, right_camera_name, right_camera_topic, right_info_camera_topic, multisenseHandoffFrame, baseTfName, right_frame_name, MULTISENSE_SL_RIGHT_CAMERA_ID);
         lidarParameters[MULTISENSE_LIDAR_ID] = new DRCRobotLidarParameters(true, lidarSensorName, multisense_filtered_laser_as_point_cloud_topic_string,
                 multisense_ground_point_cloud_topic_string, lidarJointName, lidarJointTopic, multisenseHandoffFrame, lidarBaseFrame, lidarEndFrame, lidar_spindle_velocity, MULTISENSE_LIDAR_ID);
         pointCloudParameters[MULTISENSE_STEREO_ID] = new DRCRobotPointCloudParameters(stereoSensorName, stereoColorTopic, multisenseHandoffFrame, stereoBaseFrame, stereoEndFrame, MULTISENSE_STEREO_ID);
      }
      else if (target == DRCRobotModel.RobotTarget.GAZEBO)
      {
         String baseTfName = "head";
         String left_frame_name = "left_camera_frame";
         String right_frame_name = "right_camera_frame";
         String lidarBaseFrame = "head";
         String lidarEndFrame = "head_hokuyo_frame";


         cameraParameters[MULTISENSE_SL_LEFT_CAMERA_ID] = new DRCRobotCameraParameters(RobotSide.LEFT, left_camera_name, left_camera_topic, left_info_camera_topic,
               multisenseHandoffFrame, baseTfName, left_frame_name, MULTISENSE_SL_LEFT_CAMERA_ID);
         cameraParameters[MULTISENSE_SL_RIGHT_CAMERA_ID] = new DRCRobotCameraParameters(RobotSide.RIGHT, right_camera_name, right_camera_topic, right_info_camera_topic,
               multisenseHandoffFrame, baseTfName, right_frame_name, MULTISENSE_SL_RIGHT_CAMERA_ID);
         lidarParameters[MULTISENSE_LIDAR_ID] = new DRCRobotLidarParameters(true, lidarSensorName, multisense_laser_topic_string, multisense_laser_topic_string,
               lidarJointName, lidarJointTopic, multisenseHandoffFrame, lidarBaseFrame, lidarEndFrame, lidar_spindle_velocity, MULTISENSE_LIDAR_ID);
         pointCloudParameters[MULTISENSE_STEREO_ID] = new DRCRobotPointCloudParameters(stereoSensorName, stereoColorTopic, multisenseHandoffFrame,
               stereoBaseFrame, stereoEndFrame, MULTISENSE_STEREO_ID);
      }
      else
      {
         cameraParameters[MULTISENSE_SL_LEFT_CAMERA_ID] = new DRCRobotCameraParameters(RobotSide.LEFT, left_camera_name, left_camera_topic, multisenseHandoffFrame, left_info_camera_topic, MULTISENSE_SL_LEFT_CAMERA_ID);
         cameraParameters[MULTISENSE_SL_RIGHT_CAMERA_ID] = new DRCRobotCameraParameters(RobotSide.RIGHT, right_camera_name, right_camera_topic,  multisenseHandoffFrame, right_info_camera_topic, MULTISENSE_SL_RIGHT_CAMERA_ID);
         lidarParameters[MULTISENSE_LIDAR_ID] = new DRCRobotLidarParameters(false, lidarSensorName, multisense_laser_topic_string, multisense_laser_topic_string,
               lidarJointName, lidarJointTopic, lidarPoseLink, multisenseHandoffFrame, lidarEndFrameInSdf, lidar_spindle_velocity, MULTISENSE_LIDAR_ID);
         pointCloudParameters[MULTISENSE_STEREO_ID] = new DRCRobotPointCloudParameters(stereoSensorName, stereoColorTopic, multisenseHandoffFrame, MULTISENSE_STEREO_ID);
      }

      setupHeadIMUFrames();
      cameraParameters[BLACKFLY_LEFT_CAMERA_ID] = new DRCRobotCameraParameters(RobotSide.LEFT, leftFisheyeCameraName, fisheye_left_camera_topic, fisheye_pose_source, fisheye_left_camera_info, BLACKFLY_LEFT_CAMERA_ID);
      cameraParameters[BLACKFLY_RIGHT_CAMERA_ID] = new DRCRobotCameraParameters(RobotSide.RIGHT, right_fisheye_camera_name, fisheye_right_camera_topic, fisheye_pose_source, fisheye_right_camera_info, BLACKFLY_RIGHT_CAMERA_ID);

      setupROSLocationService = target == DRCRobotModel.RobotTarget.REAL_ROBOT || (target == DRCRobotModel.RobotTarget.SCS && SEND_ROBOT_DATA_TO_ROS);
      setupROSParameterSetters = target == DRCRobotModel.RobotTarget.REAL_ROBOT;
      isMultisenseHead = target == DRCRobotModel.RobotTarget.REAL_ROBOT;

      setupStaticTransformsForRos();
	}

	private void setupStaticTransformsForRos()
   {
	   ImmutableTriple<String, String, RigidBodyTransform> headToHeadRootStaticTransform = new ImmutableTriple<String, String, RigidBodyTransform>("head", "multisense/head_root", new RigidBodyTransform());
      staticTranformsForRos.add(headToHeadRootStaticTransform);
   }

   private void setupHeadIMUFrames() {
		for (DRCRobotModel.RobotTarget target : DRCRobotModel.RobotTarget.values()) {
			Matrix3d headIMUBasisWhenLevel;
			if (target == DRCRobotModel.RobotTarget.REAL_ROBOT) {
				// each column is the unit vector of X,Y,Z axis in world frame
				headIMUBasisWhenLevel = new Matrix3d( 0, 0, 1,
						                              0,  1, 0,
						                             -1,  0, 0);

			} else {
				headIMUBasisWhenLevel = new Matrix3d(1, 0, 0, 0, 1, 0, 0, 0, 1);

			}
			headIMUFramesWhenLevel.put(target, ReferenceFrame .constructBodyFrameWithUnchangingTransformToParent(
							"head_imu", ReferenceFrame.getWorldFrame(),
							new RigidBodyTransform(headIMUBasisWhenLevel, new Vector3d())));
		}

	}

   @Override
   public DRCRobotLidarParameters[] getLidarParameters()
   {
      return lidarParameters;
   }

   @Override
   public DRCRobotLidarParameters getLidarParameters(int sensorId)
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
   public DRCRobotCameraParameters[] getCameraParameters()
   {
      return cameraParameters;
   }

   @Override
   public DRCRobotCameraParameters getCameraParameters(int sensorId)
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
   public DRCRobotPointCloudParameters[] getPointCloudParameters()
   {
      return pointCloudParameters;
   }

   @Override
   public DRCRobotPointCloudParameters getPointCloudParameters(int sensorId)
   {
      return pointCloudParameters[sensorId];
   }

   private void sensorFramesToTrack(DRCRobotSensorParameters[] sensorParams, ArrayList<String> holder)
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
   public ReferenceFrame getHeadIMUFrameWhenLevel() {
	   return headIMUFramesWhenLevel.get(target);

	}

	public static EnumMap<DRCRobotModel.RobotTarget, ReferenceFrame> getHeadIMUFramesWhenLevel() {
		return headIMUFramesWhenLevel;
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
