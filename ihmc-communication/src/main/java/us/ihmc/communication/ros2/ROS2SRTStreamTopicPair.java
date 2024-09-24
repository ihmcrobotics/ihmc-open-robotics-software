package us.ihmc.communication.ros2;

import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.SRTStreamStatus;
import us.ihmc.ros2.ROS2Topic;

public record ROS2SRTStreamTopicPair(ROS2Topic<SRTStreamStatus> streamStatusTopic, ROS2Topic<ImageMessage> imageMessageTopic, ImageType imageType)
{
   public enum ImageType
   {
      COLOR,   // 24 bit color containing R, G, and B values (not necessarily in that order)
      DEPTH    // 16 bit depth (same as 16 bit monochrome)
   }
}
