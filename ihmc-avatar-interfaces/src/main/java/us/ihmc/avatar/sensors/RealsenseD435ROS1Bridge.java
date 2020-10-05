package us.ihmc.avatar.sensors;

import sensor_msgs.msg.dds.CompressedImage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.net.URI;

public class RealsenseD435ROS1Bridge extends AbstractRosTopicSubscriber<sensor_msgs.CompressedImage>
{
   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "imagePublisherNode");

   private final IHMCROS2Publisher<CompressedImage> imagePublisher;

   public RealsenseD435ROS1Bridge()
   {
      super(sensor_msgs.CompressedImage._TYPE);

      URI masterURI = NetworkParameters.getROSURI();
      RosMainNode rosMainNode = new RosMainNode(masterURI, "ImagePublisher", true);
      rosMainNode.attachSubscriber("/depthcam/color/image_raw/compressed", this);
      rosMainNode.execute();

      ROS2Topic<CompressedImage> topic = ROS2Tools.D435_VIDEO;
      LogTools.info("Publishing to {}", topic.getName());
      imagePublisher = ROS2Tools.createPublisher(ros2Node, topic, ROS2QosProfile.DEFAULT());
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

         imagePublisher.publish(ros2Image);
      }
      catch (Exception e)
      {
         LogTools.error(e.getMessage());
         e.printStackTrace();
      }
   }

   public static void main(String[] args)
   {
      new RealsenseD435ROS1Bridge();
   }
}
