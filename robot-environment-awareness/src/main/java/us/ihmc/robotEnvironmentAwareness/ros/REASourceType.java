package us.ihmc.robotEnvironmentAwareness.ros;

public enum REASourceType
{
   LIDAR_SCAN, STEREO_POINT_CLOUD, DEPTH_POINT_CLOUD;

   public String getTopicName()
   {
      switch (this)
      {
      case LIDAR_SCAN:
         return "/ihmc/lidar_scan";
      case STEREO_POINT_CLOUD:
         return "/ihmc/stereo_vision_point_cloud";
      case DEPTH_POINT_CLOUD:
         return "/ihmc/stereo_vision_point_cloud_D435";
      default:
         throw new RuntimeException("Unexpected " + getClass().getSimpleName() + " value: " + this);
      }
   }
}