package us.ihmc.valkyrie.paramaters;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.robotSide.SideDependentList;

public class ValkyrieSensorInformation implements DRCRobotSensorInformation
{

   public static final String[] forceSensorNames = { "LeftAnkle", "RightAnkle", "LeftForearmSupinator", "RightForearmSupinator" };
   public static final SideDependentList<String> feetForceSensorNames = new SideDependentList<String>("LeftAnkle", "RightAnkle");
   public static final SideDependentList<String> wristForceSensorNames = new SideDependentList<String>("LeftForearmSupinator", "RightForearmSupinator");
   public static final String lidarSensorName = "/v1/Ibeo_sensor";
   public static final String lidarJointName = "";
   public static final String leftCameraName = "/v1/LeftHazardCamera___default__";
   public static final String rightCameraName ="/v1/RightHazardCamera___default__";
   public static final String rightTrunkIMUSensor = "v1Trunk_RightIMU";
   public static final String leftTrunkIMUSensor = "v1Trunk_LeftIMU";
   public static final String leftPelvisIMUSensor = "v1Pelvis_LeftIMU";
   public static final String rightPelvisIMUSensor = "v1Pelvis_RightIMU";
   public static final String fakePelvisIMUSensor = "v1Pelvis_SimulatedIMU";
   
   // Use this until sim can handle multiple IMUs
   // public static final String[] imuSensorsToUse = {leftPelvisIMUSensor, rightPelvisIMUSensor};
   public static final String[] imuSensorsToUse = {leftPelvisIMUSensor};
   
   @Override
   public String getLeftCameraName()
   {
      return leftCameraName;
   }

   @Override
   public String getRightCameraName()
   {
      return rightCameraName;
   }

   @Override
   public String getLidarSensorName()
   {
      return lidarSensorName;
   }

   @Override
   public String[] getIMUSensorsToUse()
   {
      return imuSensorsToUse;
   }

   @Override
   public String getLidarJointName()
   {
      return lidarJointName;
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
      return leftPelvisIMUSensor;
   }

}
