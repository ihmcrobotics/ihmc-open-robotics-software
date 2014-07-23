package us.ihmc.acsell.parameters;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotCameraParamaters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotLidarParamaters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPointCloudParamaters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotSensorInformation;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;

public class BonoSensorInformation implements DRCRobotSensorInformation
{
   
   public static final String lidarSensorName = null;
   public static final String leftCameraName = null;
   public static final String rightCameraName = null;
   public static final String imuSensor = "pelvis_pelvisIMU";
   public static final String[] imuSensorsToUse = {imuSensor};
   private final String[] forceSensorNames;
   private final SideDependentList<String> feetForceSensorNames = new SideDependentList<String>();
   
   public BonoSensorInformation()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         String robotSideLowerCaseFirstLetter = robotSide.getSideNameFirstLetter().toLowerCase();
         feetForceSensorNames.put(robotSide, robotSideLowerCaseFirstLetter + "_leg_lax");
      }     
      forceSensorNames= new String[]{feetForceSensorNames.get(RobotSide.LEFT), feetForceSensorNames.get(RobotSide.RIGHT)};
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
   public String[] getIMUSensorsToUse()
   {
      return imuSensorsToUse;
   }

   @Override
   public SideDependentList<String> getWristForceSensorNames()
   {
      return new SideDependentList<String>();
   }

   @Override
   public String getPrimaryBodyImu()
   {
      return imuSensor;
   }

   @Override
   public DRCRobotCameraParamaters[] getCameraParameters()
   {
      return new DRCRobotCameraParamaters[0];
   }

   @Override
   public DRCRobotCameraParamaters getPrimaryCameraParameters()
   {
      return null;
   }

   @Override
   public DRCRobotLidarParamaters[] getLidarParameters()
   {
      return null;
   }

   @Override
   public DRCRobotLidarParamaters getPrimaryLidarParameters()
   {
      return null;
   }

   @Override
   public DRCRobotPointCloudParamaters[] getPointCloudParameters()
   {
      return null;
   }

   @Override
   public DRCRobotPointCloudParamaters getPrimaryPointCloudParameters()
   {
      return null;
   }
}
