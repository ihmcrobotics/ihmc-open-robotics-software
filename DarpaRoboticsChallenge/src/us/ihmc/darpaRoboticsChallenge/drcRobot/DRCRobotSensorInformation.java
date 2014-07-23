package us.ihmc.darpaRoboticsChallenge.drcRobot;

import us.ihmc.robotSide.SideDependentList;

public interface DRCRobotSensorInformation
{
   public abstract String[] getIMUSensorsToUse();

   public abstract String[] getForceSensorNames();

   public abstract SideDependentList<String> getFeetForceSensorNames();
   
   public abstract SideDependentList<String> getWristForceSensorNames();

   public abstract String getPrimaryBodyImu();
   
   public abstract DRCRobotCameraParameters[] getCameraParameters();
   
   public abstract DRCRobotCameraParameters getPrimaryCameraParameters();

   public abstract DRCRobotLidarParamaters[] getLidarParameters();

   public abstract DRCRobotLidarParamaters getPrimaryLidarParameters();
   
   public abstract DRCRobotPointCloudParamaters[] getPointCloudParameters();
   
   public abstract DRCRobotPointCloudParamaters getPrimaryPointCloudParameters();
}
