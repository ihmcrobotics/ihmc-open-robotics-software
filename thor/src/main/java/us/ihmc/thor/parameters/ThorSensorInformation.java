package us.ihmc.thor.parameters;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;

import org.apache.commons.lang3.tuple.ImmutableTriple;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.rotationConversion.QuaternionConversion;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ContactSensorType;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotPointCloudParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorParameters;


// // FIXME: 11/19/16  all the sensors need to be set up

public class ThorSensorInformation implements DRCRobotSensorInformation
{

   public static final String[] forceSensorNames;
   private static final SideDependentList<String> feetForceSensorNames;
   private final ArrayList<ImmutableTriple<String, String, RigidBodyTransform>> staticTranformsForRos = new ArrayList<ImmutableTriple<String,String,RigidBodyTransform>>();

   static
   {
      feetForceSensorNames = new SideDependentList<String>("l_ankle_roll", "r_ankle_roll");
      forceSensorNames = new String[] { "l_ankle_roll", "r_ankle_roll" };
   }
   private static final SideDependentList<String> wristForceSensorNames = null; //new SideDependentList<String>("leftWristPitch", "rightWristPitch");
   private static final SideDependentList<String> footContactSensorNames = new SideDependentList<String>("l_foot_contact_sensor","r_foot_contact_sensor");
   private static final SideDependentList<String> urdfFeetForceSensorNames = new SideDependentList<>("l_foot_contact_sensor", "r_foot_contact_sensor");
   public static final SideDependentList<LinkedHashMap<String, LinkedHashMap<String,ContactSensorType>>> contactSensors = new SideDependentList<LinkedHashMap<String,LinkedHashMap<String,ContactSensorType>>>();

   public static final SideDependentList<RigidBodyTransform> transformFromSixAxisMeasurementToAnkleZUpFrames = new SideDependentList<>();
   static
   {
      RigidBodyTransform translateForwardAndDownOnFoot = new RigidBodyTransform();
      translateForwardAndDownOnFoot.setTranslation(0.021564, 0.0, -0.051054);
      translateForwardAndDownOnFoot.setRotationEulerAndZeroTranslation(Math.PI, 0.0, 0.0);

      transformFromSixAxisMeasurementToAnkleZUpFrames.put(RobotSide.LEFT, translateForwardAndDownOnFoot);
      transformFromSixAxisMeasurementToAnkleZUpFrames.put(RobotSide.RIGHT, new RigidBodyTransform(translateForwardAndDownOnFoot));
   }

   static
   {
      contactSensors.put(RobotSide.LEFT, new LinkedHashMap<String, LinkedHashMap<String, ContactSensorType>>());
      contactSensors.put(RobotSide.RIGHT, new LinkedHashMap<String, LinkedHashMap<String, ContactSensorType>>());

      contactSensors.get(RobotSide.LEFT).put("l_ankle_roll",new LinkedHashMap<String, ContactSensorType>());
      contactSensors.get(RobotSide.LEFT).get("l_ankle_roll").put(footContactSensorNames.get(RobotSide.LEFT), ContactSensorType.SOLE);

      contactSensors.get(RobotSide.RIGHT).put("r_ankle_roll",new LinkedHashMap<String, ContactSensorType>());
      contactSensors.get(RobotSide.RIGHT).get("r_ankle_roll").put(footContactSensorNames.get(RobotSide.RIGHT), ContactSensorType.SOLE);

   }

   /**
    * PointCloud Parameters
    */
   //Make pointCloudParameters null to not use point cloud in UI.
   private final DRCRobotPointCloudParameters[] pointCloudParamaters = new DRCRobotPointCloudParameters[1];
   public static final int POINT_CLOUD_SENSOR_ID = 0;

   /**
    * Multisense SL Parameters
    */

   public static int MULTISENSE_SL_LEFT_CAMERA_ID = 0;
   public static int MULTISENSE_SL_RIGHT_CAMERA_ID = 1;
//   public static int LEFT_HAZARD_CAMERA_ID = 2;
//   public static int RIGHT_HAZARD_CAMERA_ID = 3;
   public static int MULTISENSE_LIDAR_ID = 0;
   public static int MULTISENSE_STEREO_ID = 0;

   private static final String multisense_namespace = "/multisense";

   private static final String left_frame_name = multisense_namespace + "/left_camera_frame";
   private static final String right_frame_name = multisense_namespace + "/right_camera_frame";

   private final DRCRobotCameraParameters[] cameraParamaters = new DRCRobotCameraParameters[2];

   private static final String left_camera_name = "stereo_camera_left";
   private static final String right_camera_name = "stereo_camera_right";

   private static final String left_camera_topic = multisense_namespace + "/left/image_rect_color";
   private static final String left_camera_compressed_topic = left_camera_topic + "/compressed";
   private static final String left_info_camera_topic = multisense_namespace +"/left/image_rect_color/camera_info";//left/image_rect_color/camera_info

   private static final String right_camera_topic = multisense_namespace + "/right/image_rect_color";
   private static final String right_camera_compressed_topic = right_camera_topic + "/compressed";
   private static final String right_info_camera_topic = multisense_namespace +"/right/image_rect_color/camera_info";//right/image_rect_color/camera_info


//   private static final String leftStereoCameraName = "/v1/leftHazardCamera___default__";
//   private static final String leftCameraTopic = "/v1/leftHazardCamera/compressed";
//
//   private static final String rightStereoCameraName ="/v1/rightHazardCamera___default__";
//   private static final String rightCameraTopic = "/v1/rightHazardCamera/compressed";

   private static final String stereoSensorName = "stereo_camera";
   private static final String stereoColorTopic = multisense_namespace + "image_points2_color";
   private static final String stereoBaseFrame = multisense_namespace + "/head";
   private static final String stereoEndFrame = multisense_namespace + "/left_camera_frame";

   /**
    * LIDAR Parameters
    */
   private final DRCRobotLidarParameters[] lidarParamaters = new DRCRobotLidarParameters[1];

   private static final double lidar_spindle_velocity = 2.183;

   private static final String lidarPoseLink = "head_hokuyo_link";
   private static final String lidarJointName = "head_hokuyo_joint";
   private static final String lidarBaseFrame = multisense_namespace + "/head_root";
   private static final String lidarEndFrame = "/head_hokuyo_frame";
   private static final String baseTfName = "head";

   private ImmutableTriple<String, String, RigidBodyTransform> neckToLeftCameraTransform = new ImmutableTriple<>(baseTfName, multisense_namespace + "left_camera_optical_frame", new RigidBodyTransform(new Quaternion(0.997858923235, -4.00478664636e-18, -0.0654031292802, -6.1101236817e-17), new Vector3D(0.183847013385, -0.035, 0.0773367157227)));
   private ImmutableTriple<String, String, RigidBodyTransform> neckToRightCameraTransform = new ImmutableTriple<>(baseTfName, multisense_namespace + "left_camera_optical_frame", new RigidBodyTransform(new Quaternion(0.997858923235, -4.00478664636e-18, -0.0654031292802, -6.1101236817e-17), new Vector3D(0.183847013385, 0.035, 0.0773367157227)));

   private static final String lidarSensorName = "head_hokuyo_sensor";
   private static final String lidarJointTopic = multisense_namespace + "/joint_states";
   private static final String multisense_laser_topic_string = multisense_namespace+"/lidar_scan";
   private static final String multisense_near_Scan = multisense_namespace+"/filtered_cloud";
   private static final String multisense_height_map = multisense_namespace+"/highly_filtered_cloud";
   private static final String multisenseHandoffFrame = "upperNeckPitchLink";

   private static final String pelvisIMUSensor = "pelvis_imu_sensor";

   private static final HashMap<String, Integer> imuUSBSerialIds = new HashMap<>();
   static
   {
	   imuUSBSerialIds.put(pelvisIMUSensor, 422047095);
   }

   // Use this until sim can handle multiple IMUs
   public static final String[] imuSensorsToUse = {pelvisIMUSensor};

   public ThorSensorInformation(RobotTarget target)
   {
      if(target == RobotTarget.REAL_ROBOT)
      {
         lidarParamaters[MULTISENSE_LIDAR_ID] = new DRCRobotLidarParameters(true, lidarSensorName, multisense_near_Scan, multisense_height_map,
               lidarJointName, lidarJointTopic, multisenseHandoffFrame, lidarBaseFrame, lidarEndFrame, lidar_spindle_velocity, MULTISENSE_LIDAR_ID);
         cameraParamaters[MULTISENSE_SL_LEFT_CAMERA_ID] = new DRCRobotCameraParameters(RobotSide.LEFT, left_camera_name, left_camera_compressed_topic, multisenseHandoffFrame, left_info_camera_topic, neckToLeftCameraTransform.right, MULTISENSE_SL_LEFT_CAMERA_ID);

         cameraParamaters[MULTISENSE_SL_RIGHT_CAMERA_ID] = new DRCRobotCameraParameters(RobotSide.RIGHT, right_camera_name, right_camera_compressed_topic, multisenseHandoffFrame, right_info_camera_topic, neckToRightCameraTransform.right, MULTISENSE_SL_RIGHT_CAMERA_ID);

         pointCloudParamaters[MULTISENSE_STEREO_ID] = new DRCRobotPointCloudParameters(stereoSensorName, stereoColorTopic, multisenseHandoffFrame, stereoBaseFrame, stereoEndFrame, MULTISENSE_STEREO_ID);
      }
      else
      {
         lidarParamaters[MULTISENSE_LIDAR_ID] = new DRCRobotLidarParameters(false, lidarSensorName, multisense_laser_topic_string, multisense_laser_topic_string,
               lidarJointName, lidarJointTopic, multisenseHandoffFrame, lidarBaseFrame, lidarEndFrame, lidar_spindle_velocity, MULTISENSE_LIDAR_ID);
         cameraParamaters[MULTISENSE_SL_LEFT_CAMERA_ID] = new DRCRobotCameraParameters(RobotSide.LEFT, left_camera_name, left_camera_compressed_topic, left_info_camera_topic, multisenseHandoffFrame, baseTfName,
               left_frame_name, MULTISENSE_SL_LEFT_CAMERA_ID);
         cameraParamaters[MULTISENSE_SL_RIGHT_CAMERA_ID] = new DRCRobotCameraParameters(RobotSide.RIGHT, right_camera_name, right_camera_compressed_topic, right_info_camera_topic, multisenseHandoffFrame, baseTfName,
               right_frame_name, MULTISENSE_SL_RIGHT_CAMERA_ID);
         pointCloudParamaters[MULTISENSE_STEREO_ID] = new DRCRobotPointCloudParameters(stereoSensorName, stereoColorTopic, multisenseHandoffFrame, stereoBaseFrame, stereoEndFrame, MULTISENSE_STEREO_ID);
      }
      setupStaticTransformsForRos();
   }

   public static String getUrdfFeetForceSensorName(RobotSide side)
   {
      return urdfFeetForceSensorNames.get(side);
   }

   public HashMap<String, Integer> getImuUSBSerialIds()
   {
      return imuUSBSerialIds;
   }

   @Override
   public String[] getIMUSensorsToUseInStateEstimator()
   {
      return imuSensorsToUse;
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
      return wristForceSensorNames;
   }

   @Override
   public String getPrimaryBodyImu()
   {
      return pelvisIMUSensor;
   }

   @Override
   public DRCRobotCameraParameters[] getCameraParameters()
   {
      return cameraParamaters;
   }

   @Override
   public DRCRobotCameraParameters getCameraParameters(int sensorId)
   {
      return cameraParamaters[sensorId];
   }

   @Override
   public DRCRobotLidarParameters[] getLidarParameters()
   {
      return lidarParamaters;
   }

   @Override
   public DRCRobotLidarParameters getLidarParameters(int sensorId)
   {
      return lidarParamaters[sensorId];
   }

   @Override
   public DRCRobotPointCloudParameters[] getPointCloudParameters()
   {
      return pointCloudParamaters;
   }

   @Override
   public DRCRobotPointCloudParameters getPointCloudParameters(int sensorId)
   {
      return pointCloudParamaters[sensorId];
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
      sensorFramesToTrack(cameraParamaters,sensorFramesToTrack);
      sensorFramesToTrack(lidarParamaters,sensorFramesToTrack);
      String[] sensorFramesToTrackAsPrimitive = new String[sensorFramesToTrack.size()];
      sensorFramesToTrack.toArray(sensorFramesToTrackAsPrimitive);
      return sensorFramesToTrackAsPrimitive;
   }

   @Override
   public boolean setupROSLocationService()
   {
      return false;
   }

   @Override
   public boolean setupROSParameterSetters()
   {
      return false;
   }

   @Override
   public boolean isMultisenseHead()
   {
      return true;
   }

   @Override
   public ReferenceFrame getHeadIMUFrameWhenLevel()
   {
      return null;
   }

   @Override
   public SideDependentList<String> getFeetContactSensorNames()
   {
      return footContactSensorNames;
   }

   @Override
   public ArrayList<ImmutableTriple<String, String, RigidBodyTransform>> getStaticTransformsForRos()
   {
      return staticTranformsForRos;
   }

   private void setupStaticTransformsForRos()
   {
      Quaternion orientation = new Quaternion();
      Vector3D translation = new Vector3D(0.183585961, 0.0, 0.075353826);
      QuaternionConversion.convertYawPitchRollToQuaternion(0.0, 0.130899694, -Math.PI, orientation);
      RigidBodyTransform staticTransform = new RigidBodyTransform(orientation, translation);
      ImmutableTriple<String, String, RigidBodyTransform> headToHeadRootStaticTransform = new ImmutableTriple<String, String, RigidBodyTransform>("upperNeckPitchLink", "multisense/head_root",
            staticTransform);
      staticTranformsForRos.add(headToHeadRootStaticTransform);
   }
}
