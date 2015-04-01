package us.ihmc.sensorProcessing.parameters;

import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.SideDependentList;

public interface DRCRobotSensorInformation
{
   public String[] getIMUSensorsToUseInStateEstimator();

   public String[] getForceSensorNames();

   public SideDependentList<String> getFeetForceSensorNames();
   
   public SideDependentList<String> getFeetContactSensorNames();
   
   public SideDependentList<String> getWristForceSensorNames();

   public String getPrimaryBodyImu();
   
   public DRCRobotCameraParameters[] getCameraParameters();
   
   public DRCRobotLidarParameters[] getLidarParameters();

   public DRCRobotPointCloudParameters[] getPointCloudParameters();
   
   public DRCRobotCameraParameters getCameraParameters(int cameraId);

   public DRCRobotLidarParameters getLidarParameters(int lidarId);

   public DRCRobotPointCloudParameters getPointCloudParameters(int pointCloudSensorId);
   
   public ReferenceFrame getHeadIMUFrameWhenLevel();

   public String[] getSensorFramesToTrack();
   
   public boolean setupROSLocationService();

   public boolean setupROSParameterSetters();
   
   public boolean isMultisenseHead();
}
