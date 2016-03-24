package us.ihmc.sensorProcessing.parameters;

public interface RGBDCameraParameters
{
   public DRCRobotCameraParameters getCameraParameters();

   public DRCRobotPointCloudParameters getPointCloudParameters();
}
