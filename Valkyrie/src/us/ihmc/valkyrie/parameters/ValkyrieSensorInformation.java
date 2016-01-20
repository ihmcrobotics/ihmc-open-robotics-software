package us.ihmc.valkyrie.parameters;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;

import javax.vecmath.Quat4d;

import org.apache.commons.lang3.tuple.ImmutableTriple;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ContactSensorType;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotPointCloudParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorParameters;
import us.ihmc.valkyrie.configuration.ValkyrieConfigurationRoot;

public class ValkyrieSensorInformation implements DRCRobotSensorInformation
{

   public static final String[] forceSensorNames;
   private static final SideDependentList<String> feetForceSensorNames;
   private final ArrayList<ImmutableTriple<String, String, RigidBodyTransform>> staticTranformsForRos = new ArrayList<ImmutableTriple<String,String,RigidBodyTransform>>();

   static
   {
      feetForceSensorNames = new SideDependentList<String>("leftAnkleRoll", "rightAnkleRoll");
      
      if (ValkyrieConfigurationRoot.VALKYRIE_WITH_ARMS)
      {
         forceSensorNames = new String[] { "leftAnkleRoll", "rightAnkleRoll" }; //, "leftWristPitch", "rightWristPitch" };
      }
      else
      {
         forceSensorNames = new String[] { "leftAnkleRoll", "rightAnkleRoll" };
      }
   }
   private static final SideDependentList<String> wristForceSensorNames = null; //new SideDependentList<String>("leftWristPitch", "rightWristPitch");
   private static final SideDependentList<String> urdfTekscanSensorNames = new SideDependentList<String>("leftCOP_Offset", "rightCOP_Offset");
   private static final SideDependentList<String> footContactSensorNames = new SideDependentList<String>("leftFootContactSensor","rightFootContactSensor");
   private static final SideDependentList<String> urdfFeetForceSensorNames = new SideDependentList<>("leftFootSixAxis_Offset", "rightFootSixAxis_Offset");
   public static final SideDependentList<LinkedHashMap<String, LinkedHashMap<String,ContactSensorType>>> contactSensors = new SideDependentList<LinkedHashMap<String,LinkedHashMap<String,ContactSensorType>>>();

   public static final boolean USE_JSC_FOOT_MASS_TARING = false;

   public static final SideDependentList<RigidBodyTransform> transformFromSixAxisMeasurementToAnkleZUpFrames = new SideDependentList<>();
   static
   {     
      RigidBodyTransform translateForwardAndDownOnFoot = new RigidBodyTransform();
      translateForwardAndDownOnFoot.setTranslation(0.021564, 0.0, -0.051054);
      translateForwardAndDownOnFoot.setEuler(Math.PI, 0.0, 0.0);

      transformFromSixAxisMeasurementToAnkleZUpFrames.put(RobotSide.LEFT, translateForwardAndDownOnFoot);
      transformFromSixAxisMeasurementToAnkleZUpFrames.put(RobotSide.RIGHT, new RigidBodyTransform(translateForwardAndDownOnFoot));
   }
   
   static
   {
      contactSensors.put(RobotSide.LEFT, new LinkedHashMap<String, LinkedHashMap<String,ContactSensorType>>());
      
      contactSensors.get(RobotSide.LEFT).put("leftAnkleRoll",new LinkedHashMap<String,ContactSensorType>());
      contactSensors.get(RobotSide.LEFT).get("leftAnkleRoll").put(footContactSensorNames.get(RobotSide.LEFT), ContactSensorType.SOLE);
      
      //@TODO Need a bit more work before multiple contact sensors can be added to a single rigid body.
//      contactSensors.get(RobotSide.LEFT).get("LeftAnkle").put("LeftToeContactSensor", ContactSensorType.TOE);
//      contactSensors.get(RobotSide.LEFT).get("LeftAnkle").put("LeftHeelContactSensor", ContactSensorType.HEEL);
      contactSensors.put(RobotSide.RIGHT, new LinkedHashMap<String, LinkedHashMap<String,ContactSensorType>>());
      contactSensors.get(RobotSide.RIGHT).put("rightAnkleRoll",new LinkedHashMap<String,ContactSensorType>());
      contactSensors.get(RobotSide.RIGHT).get("rightAnkleRoll").put(footContactSensorNames.get(RobotSide.RIGHT), ContactSensorType.SOLE);
      
      //@TODO Need a bit more work before multiple contact sensors can be added to a single rigid body.      
//      contactSensors.get(RobotSide.RIGHT).get("RightAnkle").put("RightToeContactSensor", ContactSensorType.TOE);
//      contactSensors.get(RobotSide.RIGHT).get("RightAnkle").put("RightHeelContactSensor", ContactSensorType.HEEL);
   }

   /**
    * PointCloud Parameters
    */
   //Make pointCloudParameters null to not use point cloud in UI.
   private final DRCRobotPointCloudParameters[] pointCloudParamaters = new DRCRobotPointCloudParameters[1];
   public static final int POINT_CLOUD_SENSOR_ID = 0;
   
   private static int multisenseCameraId = 0;
   private static int leftHazardCameraId = 1;
   private static int rightHazardCameraId = 2;

   /**
    * Multisense SL Parameters
    */
   
   public static int MULTISENSE_SL_LEFT_CAMERA_ID = 0; 
   public static int MULTISENSE_LIDAR_ID = 0;                    
   public static int MULTISENSE_STEREO_ID = 0;        
   
   private static final String multisense_namespace = "/multisense";
   
   private static final String multisense_camera_frame_name = "multisense/left_camera_frame";
   private static final String headLinkName = "multisense_root_link";
   private final DRCRobotCameraParameters[] cameraParamaters = new DRCRobotCameraParameters[3];
   
   private static final String multisenseCameraName = "stereo_camera_left";
   
   private static final String left_camera_topic = multisense_namespace + "/left/image_rect_color";
   private static final String left_camera_compressed_topic = left_camera_topic + "/compressed";
   private static final String left_info_camera_topic = multisense_namespace +"/left/image_rect_color/camera_info";//left/image_rect_color/camera_info
   
   
   private static final String leftStereoCameraName = "/v1/leftHazardCamera___default__";
   private static final String leftCameraTopic = "/v1/leftHazardCamera/compressed";
   
   private static final String rightStereoCameraName ="/v1/rightHazardCamera___default__";
   private static final String rightCameraTopic = "/v1/rightHazardCamera/compressed";

   /**
    * LIDAR Parameters
    */
   private final DRCRobotLidarParameters[] lidarParamaters = new DRCRobotLidarParameters[1];

   private static final double lidar_spindle_velocity = 2.183;

   private static final String lidarPoseLink = "hokuyo_link";
   private static final String lidarJointName = "hokuyo_joint";
   private static final String lidarEndFrameInSdf = "/head_hokuyo_frame";

   private static final String lidarSensorName = "head_hokuyo_sensor";
   private static final String lidarJointTopic = multisense_namespace + "/joint_states";
   private static final String multisense_laser_topic_string = multisense_namespace+"/lidar_scan";
   private static final String multisense_near_Scan = multisense_namespace+"/near_scan";
   private static final String multisense_height_map = multisense_namespace+"/height_map";
   private static final String multisenseHandoffFrame = "multisense_root_link";
   
   private static final String rightTrunkIMUSensor = "torso_rightTorsoImu";
   private static final String leftTrunkIMUSensor = "torso_leftTorsoImu";
   private static final String rearPelvisIMUSensor = "pelvis_pelvisRearImu";
   private static final String middlePelvisIMUSensor = "pelvis_pelvisMiddleImu";
   private static final RigidBodyTransform transformFromHeadToCamera = new RigidBodyTransform();
   static
   {
      transformFromHeadToCamera.setRotation(new Quat4d(0.99786, -5.2082e-05, -0.065403, -0.00079462));
      transformFromHeadToCamera.setTranslation(0.18385, -0.035, 0.077337);
   }
   
   private static final HashMap<String, Integer> imuUSBSerialIds = new HashMap<>();
   static
   {
	   
      /* Unit B: imuUSBSerialIds.put(rearPelvisIMUSensor, 623347094); */
      /* Unit B: imuUSBSerialIds.put(middlePelvisIMUSensor, 623347092); */
	   
	   /*Unit C:*/ imuUSBSerialIds.put(rearPelvisIMUSensor, 422047095);
	   /*Unit C:*/ imuUSBSerialIds.put(middlePelvisIMUSensor, 422047093);
	   /*Unit C:*/ imuUSBSerialIds.put(leftTrunkIMUSensor, 623347099);
   }
   
   // Use this until sim can handle multiple IMUs
//    public static final String[] imuSensorsToUse = {leftPelvisIMUSensor, rightPelvisIMUSensor};
//   public static final String[] imuSensorsToUse = {rearPelvisIMUSensor};
    public static final String[] imuSensorsToUse = {rearPelvisIMUSensor, leftTrunkIMUSensor};
//   public static final String[] imuSensorsToUse = {rightPelvisIMUSensor};
   
   public ValkyrieSensorInformation(DRCRobotModel.RobotTarget target)
   {
      cameraParamaters[1] = new DRCRobotCameraParameters(RobotSide.LEFT, leftStereoCameraName,leftCameraTopic,headLinkName,leftHazardCameraId);
      cameraParamaters[2] = new DRCRobotCameraParameters(RobotSide.RIGHT, rightStereoCameraName,rightCameraTopic,headLinkName,rightHazardCameraId);

      if(target == DRCRobotModel.RobotTarget.REAL_ROBOT)
      {
         lidarParamaters[MULTISENSE_LIDAR_ID] = new DRCRobotLidarParameters(true, lidarSensorName, multisense_near_Scan, multisense_height_map,
               lidarJointName, lidarJointTopic, lidarPoseLink, multisenseHandoffFrame, lidarEndFrameInSdf, lidar_spindle_velocity, MULTISENSE_LIDAR_ID);
         cameraParamaters[0] = new DRCRobotCameraParameters(null, multisenseCameraName,left_camera_compressed_topic,headLinkName,left_info_camera_topic,transformFromHeadToCamera, multisenseCameraId);
         setupStaticTransformsForRos();
      }
      else
      {
         lidarParamaters[MULTISENSE_LIDAR_ID] = new DRCRobotLidarParameters(false, lidarSensorName, multisense_laser_topic_string, multisense_laser_topic_string,
               lidarJointName, lidarJointTopic, lidarPoseLink, multisenseHandoffFrame, lidarEndFrameInSdf, lidar_spindle_velocity, MULTISENSE_LIDAR_ID);
         cameraParamaters[0] = new DRCRobotCameraParameters(null, multisenseCameraName,left_camera_topic,multisense_camera_frame_name,left_info_camera_topic,transformFromHeadToCamera, multisenseCameraId);
      }
   }
   
   public static String getUrdfFeetForceSensorName(RobotSide side)
   {
      return urdfFeetForceSensorNames.get(side);
   }
   
   public static String getUrdfTekscanFeetForceSensorName(RobotSide side)
   {
      return urdfTekscanSensorNames.get(side);
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
      return rearPelvisIMUSensor;//rearPelvisIMUSensor;
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

   public String getRightTrunkIMUSensor()
   {
      return rightTrunkIMUSensor;
   }

   public String getLeftTrunkIMUSensor()
   {
      return leftTrunkIMUSensor;
   }

   public static String getRearPelvisIMUSensor()
   {
      return rearPelvisIMUSensor;
   }

   public static String getMiddlePelvisIMUSensor()
   {
      return middlePelvisIMUSensor;
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
      ImmutableTriple<String, String, RigidBodyTransform> headToHeadRootStaticTransform = new ImmutableTriple<String, String, RigidBodyTransform>("multisense_root_link", "multisense/head_root", new RigidBodyTransform());
      staticTranformsForRos.add(headToHeadRootStaticTransform);
   }
}
