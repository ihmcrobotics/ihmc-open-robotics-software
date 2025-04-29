package us.ihmc.alexander;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.AvatarRobotCameraParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotLidarParameters;
import us.ihmc.sensorProcessing.parameters.AvatarRobotPointCloudParameters;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;

public class AlexanderSensorInformation implements HumanoidRobotSensorInformation
{
   protected final SideDependentList<String> feetForceSensorNames = new SideDependentList<String>();

   private final String pelvisIMUSTIMSensor = "pelvis_stim_imu";
   private final String torsoIMUSensor = "torso_imu";
   private final String leftHipXIMUSensor = "left_hip_x_imu";
   private final String leftThighIMUSensor = "left_thigh_imu";
   private final String leftShinIMUSensor = "left_shin_imu";
   private final String leftFootIMUSensor = "left_foot_imu";
   private final String rightHipXIMUSensor = "right_hip_x_imu";
   private final String rightThighIMUSensor = "right_thigh_imu";
   private final String rightShinIMUSensor = "right_shin_imu";
   private final String rightFootIMUSensor = "right_foot_imu";

   private final String[] imuSensorsToUse = {pelvisIMUSTIMSensor};

   private final String leftShoulderIMUSensor = "left_shoulder_pitch_imu";
   private final String leftBicepIMUSensor = "left_shoulder_yaw_imu";
   private final String leftForearmIMUSensor = "left_wrist_yaw_imu";
   private final String leftHandIMUSensor = null; // "left_gripper_yaw_imu";

   private final String rightShoulderIMUSensor = "right_shoulder_pitch_imu";
   private final String rightBicepIMUSensor = "right_shoulder_yaw_imu";
   private final String rightForearmIMUSensor = "right_wrist_yaw_imu";
   private final String rightHandIMUSensor = null; // "right_gripper_yaw_imu";

   private AlexanderVersion alexanderVersion;

   public AlexanderSensorInformation(AlexanderVersion alexanderVersion)
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

   public String getLeftHipXIMUSensor()
   {
      return leftHipXIMUSensor;
   }

   public String getLeftThighIMUSensor()
   {
      return leftThighIMUSensor;
   }

   public String getLeftShinIMUSensor()
   {
      return leftShinIMUSensor;
   }

   public String getLeftFootIMUSensor()
   {
      return leftFootIMUSensor;
   }

   public String getRightHipXIMUSensor()
   {
      return rightHipXIMUSensor;
   }

   public String getRightThighIMUSensor()
   {
      return rightThighIMUSensor;
   }

   public String getRightShinIMUSensor()
   {
      return rightShinIMUSensor;
   }

   public String getRightFootIMUSensor()
   {
      return rightFootIMUSensor;
   }
}
