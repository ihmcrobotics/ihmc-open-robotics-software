package us.ihmc.darpaRoboticsChallenge.drcRobot;

import us.ihmc.utilities.robotSide.SideDependentList;

public interface DRCRobotSensorInformation
{
   public String[] getIMUSensorsToUseInStateEstimator();

   public String[] getIMUSensorsToProcess();

   public String[] getForceSensorNames();

   public SideDependentList<String> getFeetForceSensorNames();
   
   public SideDependentList<String> getWristForceSensorNames();

   public String getPrimaryBodyImu();
   
   public DRCRobotCameraParameters[] getCameraParameters();
   
   public DRCRobotLidarParameters[] getLidarParameters();

   public DRCRobotPointCloudParameters[] getPointCloudParameters();
   
   public DRCRobotCameraParameters getCameraParameters(int cameraId);

   public DRCRobotLidarParameters getLidarParameters(int lidarId);

   public DRCRobotPointCloudParameters getPointCloudParameters(int pointCloudSensorId);

   public String[] getSensorFramesToTrack();
   
   public boolean setupROSLocationService();

   public boolean setupROSParameterSetters();
   
   public boolean isMultisenseHead();
}
