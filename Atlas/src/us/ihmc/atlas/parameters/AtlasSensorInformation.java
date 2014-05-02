package us.ihmc.atlas.parameters;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.robotSide.SideDependentList;

public class AtlasSensorInformation implements DRCRobotSensorInformation
{
   public static final String[] forceSensorNames = { "l_leg_akx", "r_leg_akx", "l_arm_wrx", "r_arm_wrx" };
   public static final SideDependentList<String> feetForceSensorNames = new SideDependentList<String>("l_leg_akx", "r_leg_akx");
   public static final SideDependentList<String> handForceSensorNames = new SideDependentList<String>("l_arm_wrx", "r_arm_wrx");
   
   public static final String lidarJointName = "hokuyo_joint";
   public static final String lidarSensorName = "head_hokuyo_sensor";
   public static final String leftCameraName = "stereo_camera_left";
   public static final String rightCameraName = "stereo_camera_right";
   public static final String bodyIMUSensor = "pelvis_imu_sensor";
   public static final String[] imuSensorsToUse = { bodyIMUSensor };
   
   @Override
   public String getLidarJointName()
   {
      return lidarJointName;
   }

   @Override
   public String getLeftCameraName()
   {
      return leftCameraName;
   }
   
   @Override
   public String getLidarSensorName()
   {
      return lidarSensorName;
   }

   @Override
   public String getRightCameraName()
   {
      return rightCameraName;
   }
   
   @Override
   public String[] getIMUSensorsToUse()
   {
      return imuSensorsToUse;
   }
   
   @Override
   public String getPrimaryBodyImu()
   {
      return bodyIMUSensor;
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

}
