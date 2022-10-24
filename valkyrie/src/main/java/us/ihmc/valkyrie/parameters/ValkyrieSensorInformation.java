package us.ihmc.valkyrie.parameters;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutableTriple;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ContactSensorType;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.parameters.AvatarRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotPointCloudParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotSensorParameters;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;

public class ValkyrieSensorInformation implements HumanoidRobotSensorInformation
{
   private static final String LEFT_FOOT_FORCE_TORQUE_SENSOR = "leftAnkleRoll";
   private static final String RIGHT_FOOT_FORCE_TORQUE_SENSOR = "rightAnkleRoll";
   public static final String[] forceSensorNames;
   public static final Map<String, RigidBodyTransform> forceSensorTransformMap = new HashMap<>();
   private static final SideDependentList<String> feetForceSensorNames;
   public static final SideDependentList<RigidBodyTransform> transformFromSixAxisMeasurementToAnkleZUpFrames = new SideDependentList<>();
   static
   {
      RigidBodyTransform translateForwardAndDownOnFoot = new RigidBodyTransform();
      translateForwardAndDownOnFoot.getTranslation().set(0.021564, 0.0, -0.051054);
      translateForwardAndDownOnFoot.setRotationEulerAndZeroTranslation(Math.PI, 0.0, 0.0);

      transformFromSixAxisMeasurementToAnkleZUpFrames.put(RobotSide.LEFT, translateForwardAndDownOnFoot);
      transformFromSixAxisMeasurementToAnkleZUpFrames.put(RobotSide.RIGHT, new RigidBodyTransform(translateForwardAndDownOnFoot));
   }

   static
   {
      feetForceSensorNames = new SideDependentList<>(LEFT_FOOT_FORCE_TORQUE_SENSOR, RIGHT_FOOT_FORCE_TORQUE_SENSOR);
      forceSensorNames = new String[] {LEFT_FOOT_FORCE_TORQUE_SENSOR, RIGHT_FOOT_FORCE_TORQUE_SENSOR};
      forceSensorTransformMap.put(LEFT_FOOT_FORCE_TORQUE_SENSOR, transformFromSixAxisMeasurementToAnkleZUpFrames.get(RobotSide.LEFT));
      forceSensorTransformMap.put(RIGHT_FOOT_FORCE_TORQUE_SENSOR, transformFromSixAxisMeasurementToAnkleZUpFrames.get(RobotSide.RIGHT));
   }

   private static final SideDependentList<String> wristForceSensorNames = null; //new SideDependentList<String>("leftWristPitch", "rightWristPitch");
   private static final SideDependentList<String> footContactSensorNames = new SideDependentList<>("leftFootContactSensor", "rightFootContactSensor");
   private static final SideDependentList<String> urdfFeetForceSensorNames = new SideDependentList<>("leftFootSixAxis_Offset", "rightFootSixAxis_Offset");
   public static final SideDependentList<LinkedHashMap<String, LinkedHashMap<String, ContactSensorType>>> contactSensors = new SideDependentList<>();

   private static final RigidBodyTransform transformFromHeadToUpperNeckPitchLink = new RigidBodyTransform(new YawPitchRoll(0.0, 0.130899694, -Math.PI),
                                                                                                          new Vector3D(0.183585961, 0.0, 0.075353826));

   public static final boolean USE_JSC_FOOT_MASS_TARING = false;

   static
   {
      contactSensors.put(RobotSide.LEFT, new LinkedHashMap<String, LinkedHashMap<String, ContactSensorType>>());

      contactSensors.get(RobotSide.LEFT).put(LEFT_FOOT_FORCE_TORQUE_SENSOR, new LinkedHashMap<String, ContactSensorType>());
      contactSensors.get(RobotSide.LEFT).get(LEFT_FOOT_FORCE_TORQUE_SENSOR).put(footContactSensorNames.get(RobotSide.LEFT), ContactSensorType.SOLE);

      //@TODO Need a bit more work before multiple contact sensors can be added to a single rigid body.
      //      contactSensors.get(RobotSide.LEFT).get("LeftAnkle").put("LeftToeContactSensor", ContactSensorType.TOE);
      //      contactSensors.get(RobotSide.LEFT).get("LeftAnkle").put("LeftHeelContactSensor", ContactSensorType.HEEL);
      contactSensors.put(RobotSide.RIGHT, new LinkedHashMap<String, LinkedHashMap<String, ContactSensorType>>());
      contactSensors.get(RobotSide.RIGHT).put(RIGHT_FOOT_FORCE_TORQUE_SENSOR, new LinkedHashMap<String, ContactSensorType>());
      contactSensors.get(RobotSide.RIGHT).get(RIGHT_FOOT_FORCE_TORQUE_SENSOR).put(footContactSensorNames.get(RobotSide.RIGHT), ContactSensorType.SOLE);

      //@TODO Need a bit more work before multiple contact sensors can be added to a single rigid body.
      //      contactSensors.get(RobotSide.RIGHT).get("RightAnkle").put("RightToeContactSensor", ContactSensorType.TOE);
      //      contactSensors.get(RobotSide.RIGHT).get("RightAnkle").put("RightHeelContactSensor", ContactSensorType.HEEL);
   }

   private final ArrayList<ImmutableTriple<String, String, RigidBodyTransform>> staticTranformsForRos = new ArrayList<>();
   /**
    * PointCloud Parameters
    */
   //Make pointCloudParameters null to not use point cloud in UI.
   private final AvatarRobotPointCloudParameters[] pointCloudParamaters = new AvatarRobotPointCloudParameters[1];
   public static final int POINT_CLOUD_SENSOR_ID = 0;

   /**
    * Multisense SL Parameters
    */

   public static final int MULTISENSE_SL_LEFT_CAMERA_ID = 0;
   public static final int MULTISENSE_SL_RIGHT_CAMERA_ID = 1;
   //   public static final int LEFT_HAZARD_CAMERA_ID = 2;
   //   public static final int RIGHT_HAZARD_CAMERA_ID = 3;
   public static final int MULTISENSE_LIDAR_ID = 0;
   public static final int MULTISENSE_STEREO_ID = 0;

   private static final String multisense_namespace = "/multisense";

   private static final String left_frame_name = multisense_namespace + "/left_camera_frame";
   private static final String right_frame_name = multisense_namespace + "/right_camera_frame";

   private final AvatarRobotCameraParameters[] cameraParamaters = new AvatarRobotCameraParameters[2];

   private static final String left_camera_name = "stereo_camera_left";
   private static final String right_camera_name = "stereo_camera_right";

   private static final String left_camera_topic = multisense_namespace + "/left/image_rect_color";
   private static final String left_camera_compressed_topic = left_camera_topic + "/compressed";
   private static final String left_info_camera_topic = multisense_namespace + "/left/image_rect_color/camera_info";//left/image_rect_color/camera_info

   private static final String right_camera_topic = multisense_namespace + "/right/image_rect_color";
   private static final String right_camera_compressed_topic = right_camera_topic + "/compressed";
   private static final String right_info_camera_topic = multisense_namespace + "/right/image_rect_color/camera_info";//right/image_rect_color/camera_info

   //   private static final String leftStereoCameraName = "/v1/leftHazardCamera___default__";
   //   private static final String leftCameraTopic = "/v1/leftHazardCamera/compressed";
   //
   //   private static final String rightStereoCameraName ="/v1/rightHazardCamera___default__";
   //   private static final String rightCameraTopic = "/v1/rightHazardCamera/compressed";

   private static final String stereoSensorName = "stereo_camera";
   private static final String stereoColorTopic = multisense_namespace + "/image_points2_color_world";
   private static final String stereoBaseFrame = multisense_namespace + "/head";
   private static final String stereoEndFrame = multisense_namespace + "/left_camera_frame";

   /**
    * LIDAR Parameters
    */
   private final AvatarRobotLidarParameters[] lidarParamaters = new AvatarRobotLidarParameters[1];

   private static final double lidar_spindle_velocity = 2.183;

   @SuppressWarnings("unused")
   private static final String lidarPoseLink = "hokuyo_link";
   private static final String lidarJointName = "hokuyo_joint";
   private static final String lidarBaseFrame = multisense_namespace + "/head";
   private static final String lidarEndFrame = multisense_namespace + "/head_hokuyo_frame";
   private static final String baseTfName = "upperNeckPitchLink";

   private ImmutableTriple<String, String, RigidBodyTransform> neckToLeftCameraTransform = new ImmutableTriple<>(baseTfName,
                                                                                                                 multisense_namespace
                                                                                                                       + "left_camera_optical_frame",
                                                                                                                 new RigidBodyTransform(new Quaternion(0.997858923235,
                                                                                                                                                       -4.00478664636e-18,
                                                                                                                                                       -0.0654031292802,
                                                                                                                                                       -6.1101236817e-17),
                                                                                                                                        new Vector3D(0.183847013385,
                                                                                                                                                     -0.035,
                                                                                                                                                     0.0773367157227)));
   private ImmutableTriple<String, String, RigidBodyTransform> neckToRightCameraTransform = new ImmutableTriple<>(baseTfName,
                                                                                                                  multisense_namespace
                                                                                                                        + "left_camera_optical_frame",
                                                                                                                  new RigidBodyTransform(new Quaternion(0.997858923235,
                                                                                                                                                        -4.00478664636e-18,
                                                                                                                                                        -0.0654031292802,
                                                                                                                                                        -6.1101236817e-17),
                                                                                                                                         new Vector3D(0.183847013385,
                                                                                                                                                      0.035,
                                                                                                                                                      0.0773367157227)));

   private static final String lidarSensorName = "head_hokuyo_sensor";
   private static final String lidarJointTopic = multisense_namespace + "/joint_states";
   private static final String multisense_laser_topic_string = multisense_namespace + "/lidar_scan";
   private static final String multisense_near_Scan = multisense_namespace + "/filtered_cloud";
   private static final String multisense_height_map = multisense_namespace + "/highly_filtered_cloud";
   private static final String multisenseHandoffFrame = "upperNeckPitchLink";

   private static final String rightTrunkIMUSensor = "rightTorsoImu";
   private static final String leftTrunkIMUSensor = "leftTorsoImu";
   private static final String rearPelvisIMUSensor = "pelvisRearImu";
   private static final String middlePelvisIMUSensor = "pelvisMiddleImu";

   private static final RigidBodyTransform transformChestToL515DepthCamera = new RigidBodyTransform();
   static
   {
      // TODO: Move this stuff to a file so it can be tuned and saved
      transformChestToL515DepthCamera.setIdentity();
      transformChestToL515DepthCamera.getTranslation().set(0.275000, 0.052000, 0.140000);
      transformChestToL515DepthCamera.getRotation().setYawPitchRoll(0.010000, 1.151900, 0.045000);
   }

   private static final HashMap<String, Integer> imuUSBSerialIds = new HashMap<>();
   static
   {

      /* Unit B: imuUSBSerialIds.put(rearPelvisIMUSensor, 623347094); */
      /* Unit B: imuUSBSerialIds.put(middlePelvisIMUSensor, 623347092); */

      /* Unit C: */ imuUSBSerialIds.put(rearPelvisIMUSensor, 422047095);
      /* Unit C: */ imuUSBSerialIds.put(middlePelvisIMUSensor, 422047093);
      /* Unit C: */ imuUSBSerialIds.put(leftTrunkIMUSensor, 623347099);
   }

   // Use this until sim can handle multiple IMUs
   //    public static final String[] imuSensorsToUse = {leftPelvisIMUSensor, rightPelvisIMUSensor};
   //   public static final String[] imuSensorsToUse = {rearPelvisIMUSensor};
   //    public static final String[] imuSensorsToUse = {middlePelvisIMUSensor, leftTrunkIMUSensor};
   public static final String[] imuSensorsToUse = {rearPelvisIMUSensor, leftTrunkIMUSensor};
   //   public static final String[] imuSensorsToUse = {rightPelvisIMUSensor};

   public static final double linearVelocityThreshold = 0.15;
   public static final double angularVelocityThreshold = Math.PI / 15;
   private ValkyriePhysicalProperties physicalProperties;

   public ValkyrieSensorInformation(ValkyriePhysicalProperties physicalProperties, RobotTarget target)
   {
      //      cameraParamaters[LEFT_HAZARD_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.LEFT, leftStereoCameraName,leftCameraTopic,headLinkName,leftHazardCameraId);
      //      cameraParamaters[RIGHT_HAZARD_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.RIGHT, rightStereoCameraName,rightCameraTopic,headLinkName,rightHazardCameraId);

      this.physicalProperties = physicalProperties;

      if (target == RobotTarget.REAL_ROBOT)
      {
         lidarParamaters[MULTISENSE_LIDAR_ID] = new AvatarRobotLidarParameters(true,
                                                                               lidarSensorName,
                                                                               multisense_near_Scan,
                                                                               multisense_height_map,
                                                                               lidarJointName,
                                                                               lidarJointTopic,
                                                                               multisenseHandoffFrame,
                                                                               lidarBaseFrame,
                                                                               lidarEndFrame,
                                                                               lidar_spindle_velocity,
                                                                               MULTISENSE_LIDAR_ID);
         cameraParamaters[MULTISENSE_SL_LEFT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.LEFT,
                                                                                          left_camera_name,
                                                                                          left_camera_compressed_topic,
                                                                                          multisenseHandoffFrame,
                                                                                          left_info_camera_topic,
                                                                                          neckToLeftCameraTransform.right,
                                                                                          MULTISENSE_SL_LEFT_CAMERA_ID);

         cameraParamaters[MULTISENSE_SL_RIGHT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.RIGHT,
                                                                                           right_camera_name,
                                                                                           right_camera_compressed_topic,
                                                                                           multisenseHandoffFrame,
                                                                                           right_info_camera_topic,
                                                                                           neckToRightCameraTransform.right,
                                                                                           MULTISENSE_SL_RIGHT_CAMERA_ID);

         pointCloudParamaters[MULTISENSE_STEREO_ID] = new AvatarRobotPointCloudParameters(stereoSensorName,
                                                                                          stereoColorTopic,
                                                                                          multisenseHandoffFrame,
                                                                                          stereoBaseFrame,
                                                                                          stereoEndFrame,
                                                                                          MULTISENSE_STEREO_ID);
      }
      else
      {
         lidarParamaters[MULTISENSE_LIDAR_ID] = new AvatarRobotLidarParameters(false,
                                                                               lidarSensorName,
                                                                               multisense_laser_topic_string,
                                                                               multisense_laser_topic_string,
                                                                               lidarJointName,
                                                                               lidarJointTopic,
                                                                               multisenseHandoffFrame,
                                                                               lidarBaseFrame,
                                                                               lidarEndFrame,
                                                                               lidar_spindle_velocity,
                                                                               MULTISENSE_LIDAR_ID);
         cameraParamaters[MULTISENSE_SL_LEFT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.LEFT,
                                                                                          left_camera_name,
// SCS produces an uncompressed image rather than a compressed image                                                                                          
//                                                                                          left_camera_compressed_topic,
                                                                                          left_camera_topic,
                                                                                          left_info_camera_topic,
                                                                                          multisenseHandoffFrame,
                                                                                          baseTfName,
                                                                                          left_frame_name,
                                                                                          MULTISENSE_SL_LEFT_CAMERA_ID);
         cameraParamaters[MULTISENSE_SL_RIGHT_CAMERA_ID] = new AvatarRobotCameraParameters(RobotSide.RIGHT,
                                                                                           right_camera_name,
// SCS produces an uncompressed image rather than a compressed image                                                     
//                                                                                           right_camera_compressed_topic,
                                                                                           right_camera_topic,
                                                                                           right_info_camera_topic,
                                                                                           multisenseHandoffFrame,
                                                                                           baseTfName,
                                                                                           right_frame_name,
                                                                                           MULTISENSE_SL_RIGHT_CAMERA_ID);
         pointCloudParamaters[MULTISENSE_STEREO_ID] = new AvatarRobotPointCloudParameters(stereoSensorName,
                                                                                          stereoColorTopic,
                                                                                          multisenseHandoffFrame,
                                                                                          stereoBaseFrame,
                                                                                          stereoEndFrame,
                                                                                          MULTISENSE_STEREO_ID);
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

   public static RigidBodyTransform getForceSensorTransform(String sensorName)
   {
      return forceSensorTransformMap.get(sensorName);
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
      return middlePelvisIMUSensor;//rearPelvisIMUSensor;
   }

   @Override
   public AvatarRobotCameraParameters[] getCameraParameters()
   {
      return cameraParamaters;
   }

   @Override
   public AvatarRobotCameraParameters getCameraParameters(int sensorId)
   {
      return cameraParamaters[sensorId];
   }

   @Override
   public AvatarRobotLidarParameters[] getLidarParameters()
   {
      return lidarParamaters;
   }

   @Override
   public AvatarRobotLidarParameters getLidarParameters(int sensorId)
   {
      return lidarParamaters[sensorId];
   }

   @Override
   public AvatarRobotPointCloudParameters[] getPointCloudParameters()
   {
      return pointCloudParamaters;
   }

   @Override
   public AvatarRobotPointCloudParameters getPointCloudParameters(int sensorId)
   {
      return pointCloudParamaters[sensorId];
   }

   private void sensorFramesToTrack(AvatarRobotSensorParameters[] sensorParams, ArrayList<String> holder)
   {
      for (int i = 0; i < sensorParams.length; i++)
      {
         if (sensorParams[i].getPoseFrameForSdf() != null)
         {
            holder.add(sensorParams[i].getPoseFrameForSdf());
         }
      }
   }

   @Override
   public String[] getSensorFramesToTrack()
   {
      ArrayList<String> sensorFramesToTrack = new ArrayList<>();
      sensorFramesToTrack(cameraParamaters, sensorFramesToTrack);
      sensorFramesToTrack(lidarParamaters, sensorFramesToTrack);
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

   public String getRightTrunkIMUSensor()
   {
      return rightTrunkIMUSensor;
   }

   public String getLeftTrunkIMUSensor()
   {
      return leftTrunkIMUSensor;
   }

   public String getRearPelvisIMUSensor()
   {
      return rearPelvisIMUSensor;
   }

   public String getMiddlePelvisIMUSensor()
   {
      return middlePelvisIMUSensor;
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
      RigidBodyTransform staticTransform = new RigidBodyTransform(transformFromHeadToUpperNeckPitchLink);
      staticTranformsForRos.add(new ImmutableTriple<>("upperNeckPitchLink", "multisense/head", staticTransform));

      for (RobotSide robotSide : RobotSide.values)
      {
         String footName = robotSide.getCamelCaseName() + "Foot";
         String soleName = robotSide.getCamelCaseName() + "Sole";
         RigidBodyTransform transform = new RigidBodyTransform(physicalProperties.getSoleToAnkleFrameTransform(robotSide));
         staticTranformsForRos.add(new ImmutableTriple<>(footName, soleName, transform));
      }
   }

   public static RigidBodyTransform getTransformFromHeadToUpperNeckPitchLink()
   {
      return transformFromHeadToUpperNeckPitchLink;
   }
}
