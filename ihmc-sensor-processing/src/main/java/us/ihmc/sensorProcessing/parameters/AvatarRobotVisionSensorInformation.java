package us.ihmc.sensorProcessing.parameters;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public interface AvatarRobotVisionSensorInformation
{
   public AvatarRobotCameraParameters[] getCameraParameters();

   public AvatarRobotLidarParameters[] getLidarParameters();

   public AvatarRobotPointCloudParameters[] getPointCloudParameters();

   public AvatarRobotCameraParameters getCameraParameters(int cameraId);

   public AvatarRobotLidarParameters getLidarParameters(int lidarId);

   public AvatarRobotPointCloudParameters getPointCloudParameters(int pointCloudSensorId);

   public boolean isMultisenseHead();
}
