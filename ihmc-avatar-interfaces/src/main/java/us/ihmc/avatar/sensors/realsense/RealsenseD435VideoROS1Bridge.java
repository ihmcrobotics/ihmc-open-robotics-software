package us.ihmc.avatar.sensors.realsense;

import sensor_msgs.msg.dds.CompressedImage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class RealsenseD435VideoROS1Bridge extends AbstractRosTopicSubscriber<sensor_msgs.CompressedImage>
{
   private final IHMCROS2Publisher<CompressedImage> publisher;

   public RealsenseD435VideoROS1Bridge(RosMainNode ros1Node, ROS2Node ros2Node)
   {
      super(sensor_msgs.CompressedImage._TYPE);

      String ros1Topic = "/depthcam/color/image_raw/compressed";
      LogTools.info("Subscribing ROS 1: {}", ros1Topic);
      ros1Node.attachSubscriber(ros1Topic, this);

      ROS2Topic<CompressedImage> ros2Topic = ROS2Tools.D435_VIDEO;
      LogTools.info("Publishing ROS 2: {}", ros2Topic.getName());
      publisher = ROS2Tools.createPublisher(ros2Node, ros2Topic, ROS2QosProfile.DEFAULT());
   }

   @Override
   public void onNewMessage(sensor_msgs.CompressedImage ros1Image)
   {
      try
      {
         CompressedImage ros2Image = new CompressedImage();
         byte[] data = ros1Image.getData().array();
         int dataOffset = ros1Image.getData().arrayOffset();
         int length = data.length;
         ros2Image.getData().add(data, dataOffset, length - dataOffset);

         publisher.publish(ros2Image);
      }
      catch (Exception e)
      {
         LogTools.error(e.getMessage());
         e.printStackTrace();
      }
   }
}
