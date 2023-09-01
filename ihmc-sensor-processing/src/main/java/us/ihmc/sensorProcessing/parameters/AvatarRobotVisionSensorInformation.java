package us.ihmc.sensorProcessing.parameters;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public interface AvatarRobotVisionSensorInformation
{
   AvatarRobotCameraParameters[] getCameraParameters();

   AvatarRobotLidarParameters[] getLidarParameters();

   AvatarRobotPointCloudParameters[] getPointCloudParameters();

   default AvatarRobotCameraParameters getCameraParameters(int cameraId)
   {
      if (getCameraParameters() == null)
         return null;
      else
         return getCameraParameters()[cameraId];
   }

   default AvatarRobotLidarParameters getLidarParameters(int lidarId)
   {
      if (getLidarParameters() == null)
         return null;
      else
         return getLidarParameters()[lidarId];
   }

   default AvatarRobotPointCloudParameters getPointCloudParameters(int pointCloudSensorId)
   {
      if (getPointCloudParameters() == null)
         return null;
      else
         return getPointCloudParameters()[pointCloudSensorId];
   }

   default boolean isMultisenseHead()
   {
      return false;
   }
}
