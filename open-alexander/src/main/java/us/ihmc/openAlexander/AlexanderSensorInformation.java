package us.ihmc.openAlexander;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.parameters.AvatarRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotPointCloudParameters;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensors.zed.ZEDModelData;

public class AlexanderSensorInformation implements HumanoidRobotSensorInformation
{
   // ZED X Mini
   private static final RigidBodyTransform ZED_X_MINI_TO_HEAD_TRANSFORM = new RigidBodyTransform();
   static
   {
      ZED_X_MINI_TO_HEAD_TRANSFORM.getTranslation().set(0.13041,  -0.01079,  -0.00619);
      EuclidCoreMissingTools.setYawPitchRollDegrees(ZED_X_MINI_TO_HEAD_TRANSFORM.getRotation(), 0.0, 7.87148, 0.0);
   }

   private static final SideDependentList<RigidBodyTransform> ZED_X_MINI_LENSES_TO_HEAD_TRANSFORM = new SideDependentList<RigidBodyTransform>()
   {{
      RigidBodyTransform leftTransform = new RigidBodyTransform(ZED_X_MINI_TO_HEAD_TRANSFORM);
      leftTransform.getTranslation().add(0.0, ZEDModelData.ZED_X_MINI.getCenterToCameraDistance(), 0.0);
      set(RobotSide.LEFT, leftTransform);

      RigidBodyTransform rightTransform = new RigidBodyTransform(ZED_X_MINI_TO_HEAD_TRANSFORM);
      rightTransform.getTranslation().sub(0.0, ZEDModelData.ZED_X_MINI.getCenterToCameraDistance(), 0.0);
      set(RobotSide.RIGHT, rightTransform);
   }};

   protected final SideDependentList<String> feetForceSensorNames = new SideDependentList<String>();
   protected final SideDependentList<String> feetForceSensorParentJointNames = new SideDependentList<String>();

   private final String leftShoulderIMUSensor = "left_shoulder_y_imu";
   private final String leftBicepIMUSensor = "left_shoulder_z_imu";
   private final String leftForearmIMUSensor = "left_wrist_z_imu";
   private final String leftHandIMUSensor = null; // "left_gripper_z_imu";

   private final String rightShoulderIMUSensor = "right_shoulder_y_imu";
   private final String rightBicepIMUSensor = "right_shoulder_z_imu";
   private final String rightForearmIMUSensor = "right_wrist_z_imu";
   private final String rightHandIMUSensor = null; // "right_gripper_z_imu";

   // Torso IMUs
   private final String torsoIMU = "torso_imu";

   // Pelvis IMUs
   private final String pelvisIMU = "pelvis_imu";

   // Left leg IMUs
   private final String leftHipXIMU = "left_hip_x_imu";
   private final String leftThighIMU = "left_thigh_imu";
   private final String leftShinIMU = "left_shin_imu";
   private final String leftFootIMU = "left_foot_imu";

   // Right leg IMUs
   private final String rightHipXIMU = "right_hip_x_imu";
   private final String rightThighIMU = "right_thigh_imu";
   private final String rightShinIMU = "right_shin_imu";
   private final String rightFootIMU = "right_foot_imu";

   //
   private final SideDependentList<String> hipIMUNames = new SideDependentList<>();
   private final SideDependentList<String> thighIMUNames = new SideDependentList<>();
   private final SideDependentList<String> shinIMUNames = new SideDependentList<>();
   private final SideDependentList<String> footIMUNames = new SideDependentList<>();

   // IMUs to use
   private final String[] imuSensorsToUse = {pelvisIMU,
                                             leftHipXIMU,
                                             leftThighIMU,
                                             leftShinIMU,
                                             leftFootIMU,
                                             rightHipXIMU,
                                             rightThighIMU,
                                             rightShinIMU,
                                             rightFootIMU};

   private AlexanderVersionInterface alexanderVersion;

   public AlexanderSensorInformation(AlexanderVersionInterface alexanderVersion)
   {
      this.alexanderVersion = alexanderVersion;

      feetForceSensorNames.put(RobotSide.LEFT, "LeftFootFTSensor");
      feetForceSensorNames.put(RobotSide.RIGHT, "RightFootFTSensor");
      feetForceSensorParentJointNames.put(RobotSide.LEFT, "LEFT_ANKLE_X");
      feetForceSensorParentJointNames.put(RobotSide.RIGHT, "RIGHT_ANKLE_X");

      hipIMUNames.put(RobotSide.LEFT, leftHipXIMU);
      hipIMUNames.put(RobotSide.RIGHT, rightHipXIMU);

      thighIMUNames.put(RobotSide.LEFT, leftThighIMU);
      thighIMUNames.put(RobotSide.RIGHT, rightThighIMU);

      shinIMUNames.put(RobotSide.LEFT, leftShinIMU);
      shinIMUNames.put(RobotSide.RIGHT, rightShinIMU);

      footIMUNames.put(RobotSide.LEFT, leftFootIMU);
      footIMUNames.put(RobotSide.RIGHT, rightFootIMU);
   }

   @Override
   public String[] getIMUSensorsToUseInStateEstimator()
   {
      return imuSensorsToUse;
   }

   @Override
   public AvatarRobotCameraParameters[] getCameraParameters()
   {
      return null;
   }

   @Override
   public AvatarRobotCameraParameters getCameraParameters(int sensorId)
   {
      return null;
   }

   @Override
   public AvatarRobotLidarParameters[] getLidarParameters()
   {
      return null;
   }

   @Override
   public AvatarRobotLidarParameters getLidarParameters(int sensorId)
   {
      return null;
   }

   @Override
   public AvatarRobotPointCloudParameters[] getPointCloudParameters()
   {
      return null;
   }

   @Override
   public AvatarRobotPointCloudParameters getPointCloudParameters(int sensorId)
   {
      return null;
   }

   @Override
   public String[] getForceSensorNames()
   {
      return new String[0];
   }

   @Override
   public SideDependentList<String> getFeetForceSensorNames()
   {
      return feetForceSensorNames;
   }

   public SideDependentList<String> getFeetForceSensorParentJointNames()
   {
      return feetForceSensorParentJointNames;
   }

   @Override
   public SideDependentList<String> getWristForceSensorNames()
   {
      return null;
   }

   @Override
   public String getPrimaryBodyImu()
   {
      return pelvisIMU;
   }

   public String getTorsoIMUName()
   {
      return torsoIMU;
   }

   public String getHipIMUName(RobotSide robotSide)
   {
      return hipIMUNames.get(robotSide);
   }

   public String getThighIMUName(RobotSide robotSide)
   {
      return thighIMUNames.get(robotSide);
   }

   public String getShinIMUName(RobotSide robotSide)
   {
      return shinIMUNames.get(robotSide);
   }

   public String getFootIMUName(RobotSide robotSide)
   {
      return footIMUNames.get(robotSide);
   }

   @Override
   public ReferenceFrame getStereoCameraParentFrame(RobotSide side, CommonHumanoidReferenceFrames referenceFrames)
   {
      return referenceFrames.getHeadFrame();
   }

   @Override
   public ReferenceFrame getExperimentalCameraParentFrame(CommonHumanoidReferenceFrames referenceFrames)
   {
      return referenceFrames.getHeadFrame();
   }

   @Override
   public RigidBodyTransform getExperimentalCameraTransform()
   {
      return ZED_X_MINI_TO_HEAD_TRANSFORM;
   }

   @Override
   public RigidBodyTransform getStereoCameraTransform(RobotSide side)
   {
      return ZED_X_MINI_LENSES_TO_HEAD_TRANSFORM.get(side);
   }
}
