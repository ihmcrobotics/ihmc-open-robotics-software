package us.ihmc.robotEnvironmentAwareness.ros;

import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;

public enum REASourceType
{
   LIDAR_SCAN, STEREO_POINT_CLOUD, DEPTH_POINT_CLOUD;

   public String getTopicName()
   {
      switch (this)
      {
      case LIDAR_SCAN:
         return PerceptionAPI.MULTISENSE_LIDAR_SCAN.getName();
      case STEREO_POINT_CLOUD:
         return PerceptionAPI.MULTISENSE_STEREO_POINT_CLOUD.getName();
      case DEPTH_POINT_CLOUD:
         return PerceptionAPI.D435_POINT_CLOUD.getName();
      default:
         throw new RuntimeException("Unexpected " + getClass().getSimpleName() + " value: " + this);
      }
   }

   public Topic<Boolean> getEnableTopic()
   {
      switch (this)
      {
      case LIDAR_SCAN:
         return REAModuleAPI.LidarBufferEnable;
      case STEREO_POINT_CLOUD:
         return REAModuleAPI.StereoVisionBufferEnable;
      case DEPTH_POINT_CLOUD:
         return REAModuleAPI.DepthCloudBufferEnable;
      default:
         throw new RuntimeException("Unexpected " + getClass().getSimpleName() + " value: " + this);
      }
   }
}