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
   protected final SideDependentList<String> feetForceSensorParentJointNames = new SideDependentList<String>();

   private final String leftShoulderIMUSensor = "left_shoulder_pitch_imu";
   private final String leftBicepIMUSensor = "left_shoulder_yaw_imu";
   private final String leftForearmIMUSensor = "left_wrist_yaw_imu";
   private final String leftHandIMUSensor = null; // "left_gripper_yaw_imu";

   private final String rightShoulderIMUSensor = "right_shoulder_pitch_imu";
   private final String rightBicepIMUSensor = "right_shoulder_yaw_imu";
   private final String rightForearmIMUSensor = "right_wrist_yaw_imu";
   private final String rightHandIMUSensor = null; // "right_gripper_yaw_imu";

   // Torso IMUs
   private final String torsoIMU = "torso_imu";

   // Pelvis IMUs
   private final String pelvisIMU = "pelvis_stim_imu";

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
}
