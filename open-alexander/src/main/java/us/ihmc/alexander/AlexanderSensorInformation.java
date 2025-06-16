package us.ihmc.alexander;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.AvatarRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotPointCloudParameters;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;

public class AlexanderSensorInformation implements HumanoidRobotSensorInformation
{
   private static final RigidBodyTransform ZED_2I_TO_CHEST_TRANSFORM = new RigidBodyTransform();
   static
   {
      ZED_2I_TO_CHEST_TRANSFORM.getTranslation().set(0.13251,  -0.00116,  0.18667 );
      EuclidCoreMissingTools.setYawPitchRollDegrees(ZED_2I_TO_CHEST_TRANSFORM.getRotation(), 0.0, 42.76840, 0.0);
   }

   protected final SideDependentList<String> feetForceSensorNames = new SideDependentList<String>();

   private final String pelvisIMUSTIMSensor = "pelvis_stim_imu";
   private final String torsoIMUSensor = "torso_imu";

   private final String[] imuSensorsToUse = {pelvisIMUSTIMSensor};

   private final String leftShoulderIMUSensor = "left_shoulder_pitch_imu";
   private final String leftBicepIMUSensor = "left_shoulder_yaw_imu";
   private final String leftForearmIMUSensor = "left_wrist_yaw_imu";
   private final String leftHandIMUSensor = null; // "left_gripper_yaw_imu";

   private final String rightShoulderIMUSensor = "right_shoulder_pitch_imu";
   private final String rightBicepIMUSensor = "right_shoulder_yaw_imu";
   private final String rightForearmIMUSensor = "right_wrist_yaw_imu";
   private final String rightHandIMUSensor = null; // "right_gripper_yaw_imu";

   private AlexanderVersionInterface alexanderVersion;

   public AlexanderSensorInformation(AlexanderVersionInterface alexanderVersion)
   {
      this.alexanderVersion = alexanderVersion;

      feetForceSensorNames.put(RobotSide.LEFT, "LEFT_ANKLE_X");
      feetForceSensorNames.put(RobotSide.RIGHT, "RIGHT_ANKLE_X");
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

   @Override
   public SideDependentList<String> getWristForceSensorNames()
   {
      return null;
   }

   @Override
   public String getPrimaryBodyImu()
   {
      return pelvisIMUSTIMSensor;
   }

   public String getTorsoIMUName()
   {
      return torsoIMUSensor;
   }

   @Override
   public RigidBodyTransform getExperimentalCameraTransform()
   {
      return ZED_2I_TO_CHEST_TRANSFORM;
   }
}
