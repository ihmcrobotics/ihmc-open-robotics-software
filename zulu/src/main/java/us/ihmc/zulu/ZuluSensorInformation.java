package us.ihmc.zulu;

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

import java.util.ArrayList;
import java.util.List;

public class ZuluSensorInformation implements HumanoidRobotSensorInformation
{
   // ZED X Mini
   private static final RigidBodyTransform ZED_X_MINI_TO_HEAD_TRANSFORM = new RigidBodyTransform();
   static
   {
      ZED_X_MINI_TO_HEAD_TRANSFORM.getTranslation().set(0.13041,  -0.01079,  0.02381);
      EuclidCoreMissingTools.setYawPitchRollDegrees(ZED_X_MINI_TO_HEAD_TRANSFORM.getRotation(), 0.0, 20.81529, 0.0);
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

   private static final RigidBodyTransform D457_TO_CHEST_TRANSFORM = new RigidBodyTransform();
   static
   {
      D457_TO_CHEST_TRANSFORM.getTranslation().set(0.1,  0.0,  0.05);
      EuclidCoreMissingTools.setYawPitchRollDegrees(D457_TO_CHEST_TRANSFORM.getRotation(), 0.0, 52.0, 0.0);
   }

   protected final SideDependentList<String> feetForceSensorNames = new SideDependentList<String>();
   protected final SideDependentList<String> feetForceSensorParentJointNames = new SideDependentList<String>();

   // Head IMU
   private static final String headIMU = "head_imu";

   // Left upper arm IMUs
   private static final String leftShoulderIMU = "left_shoulder_y_imu";
   private static final String leftBicepIMU = "left_shoulder_z_imu";
   // Left lower arm IMUs
   private static final String leftForearmIMU = "left_wrist_z_imu";
   private static final String leftHandIMU = "left_gripper_z_imu";

   // Right upper arm IMUs
   private static final String rightShoulderIMU = "right_shoulder_y_imu";
   private static final String rightBicepIMU = "right_shoulder_z_imu";
   // Right lower arm IMUs
   private static final String rightForearmIMU = "right_wrist_z_imu";
   private static final String rightHandIMU = "right_gripper_z_imu";

   // Torso IMUs
   private static final String torsoIMU = "torso_imu";

   // Pelvis IMUs
   private static final String pelvisIMU = "pelvis_imu";

   // Left leg IMUs
   private static final String leftHipXIMU = "left_hip_x_imu";
   private static final String leftThighIMU = "left_thigh_imu";
   private static final String leftShinIMU = "left_shin_imu";
   private static final String leftFootIMU = "left_foot_imu";

   // Right leg IMUs
   private static final String rightHipXIMU = "right_hip_x_imu";
   private static final String rightThighIMU = "right_thigh_imu";
   private static final String rightShinIMU = "right_shin_imu";
   private static final String rightFootIMU = "right_foot_imu";

   private final SideDependentList<String> hipIMUNames = new SideDependentList<>(leftHipXIMU, rightHipXIMU);
   private final SideDependentList<String> thighIMUNames = new SideDependentList<>(leftThighIMU, rightThighIMU);
   private final SideDependentList<String> shinIMUNames = new SideDependentList<>(leftShinIMU, rightShinIMU);
   private final SideDependentList<String> footIMUNames = new SideDependentList<>(leftFootIMU, rightFootIMU);

   private final SideDependentList<String> shoulderIMUNames = new SideDependentList<>(leftShoulderIMU, rightShoulderIMU);
   private final SideDependentList<String> bicepIMUNames = new SideDependentList<>(leftBicepIMU, rightBicepIMU);
   private final SideDependentList<String> forearmIMUNames = new SideDependentList<>(leftForearmIMU, rightForearmIMU);
   private final SideDependentList<String> handIMUNames = new SideDependentList<>(leftHandIMU, rightHandIMU);

   private static final String[] imuSensorsToIgnore = {leftBicepIMU, rightBicepIMU};

   // IMUs to use
   private final List<String> imuSensorsToUse = new ArrayList<>(List.of(pelvisIMU,
                                                                        torsoIMU,
                                                                        leftHipXIMU,
                                                                        leftThighIMU,
                                                                        leftShinIMU,
                                                                        leftFootIMU,
                                                                        rightHipXIMU,
                                                                        rightThighIMU,
                                                                        rightShinIMU,
                                                                        rightFootIMU,
                                                                        headIMU));

   private ZuluVersionInterface alexanderVersion;

   public ZuluSensorInformation(ZuluVersionInterface alexanderVersion)
   {
      this.alexanderVersion = alexanderVersion;

      feetForceSensorNames.put(RobotSide.LEFT, "LeftFootFTSensor");
      feetForceSensorNames.put(RobotSide.RIGHT, "RightFootFTSensor");
      feetForceSensorParentJointNames.put(RobotSide.LEFT, "LEFT_ANKLE_X");
      feetForceSensorParentJointNames.put(RobotSide.RIGHT, "RIGHT_ANKLE_X");

      if (!alexanderVersion.hasHead())
         imuSensorsToUse.remove(headIMU);

      for (RobotSide robotSide : RobotSide.values)
      {
         if (!alexanderVersion.hasArm(robotSide))
         {
            imuSensorsToUse.remove(shoulderIMUNames.get(robotSide));
            imuSensorsToUse.remove(bicepIMUNames.get(robotSide));
            imuSensorsToUse.remove(forearmIMUNames.get(robotSide));
            imuSensorsToUse.remove(handIMUNames.get(robotSide));
         }
         else if (!alexanderVersion.hasCycloidForearm())
         {
            imuSensorsToUse.remove(forearmIMUNames.get(robotSide));
            imuSensorsToUse.remove(handIMUNames.get(robotSide));
         }
      }
   }

   @Override
   public String[] getIMUSensorsToUseInStateEstimator()
   {
      return imuSensorsToUse.toArray(new String[0]);
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

   public String getHeadIMUName()
   {
      return headIMU;
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

   public String getShoulderIMUName(RobotSide robotSide)
   {
      return shoulderIMUNames.get(robotSide);
   }

   public String getBicepIMUName(RobotSide robotSide)
   {
      return bicepIMUNames.get(robotSide);
   }

   public String getForearmIMUName(RobotSide robotSide)
   {
      return forearmIMUNames.get(robotSide);
   }

   public String getHandIMUName(RobotSide robotSide)
   {
      return handIMUNames.get(robotSide);
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

   @Override
   public RigidBodyTransform getSteppingCameraTransform()
   {
      return D457_TO_CHEST_TRANSFORM;
   }

   public static String[] getImuSensorsToIgnore()
   {
      return imuSensorsToIgnore;
   }
}
