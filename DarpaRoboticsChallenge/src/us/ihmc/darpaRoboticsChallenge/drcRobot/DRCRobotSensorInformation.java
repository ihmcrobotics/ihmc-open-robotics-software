package us.ihmc.darpaRoboticsChallenge.drcRobot;

import us.ihmc.robotSide.SideDependentList;

public interface DRCRobotSensorInformation
{
   public abstract String[] getIMUSensorsToUse();

   public abstract String[] getForceSensorNames();

   public abstract SideDependentList<String> getFeetForceSensorNames();
   
   public abstract SideDependentList<String> getWristForceSensorNames();

   public abstract String getPrimaryBodyImu();
   
   public abstract DRCRobotCameraParamaters[] getCameraParamaters();
   
   public abstract DRCRobotCameraParamaters getPrimaryCameraParamaters();

   public abstract DRCRobotLidarParamaters[] getLidarParamaters();

   public abstract DRCRobotLidarParamaters getPrimaryLidarParameters();
   
   public abstract DRCRobotPointCloudParamaters[] getPointCloudParamaters();
   
   public abstract DRCRobotPointCloudParamaters getPrimaryPointCloudParameters();
}
