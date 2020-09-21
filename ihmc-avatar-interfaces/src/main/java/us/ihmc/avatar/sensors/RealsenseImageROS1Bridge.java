package us.ihmc.avatar.sensors;

import sensor_msgs.msg.dds.CompressedImage;
import sensor_msgs.msg.dds.Image;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.string.StringTools;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

public class RealsenseImageROS1Bridge extends AbstractRosTopicSubscriber<sensor_msgs.CompressedImage>
{
   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "imagePublisherNode");

   private final IHMCROS2Publisher<CompressedImage> imagePublisher;

   public RealsenseImageROS1Bridge()
   {
      super(sensor_msgs.CompressedImage._TYPE);

      URI masterURI = NetworkParameters.getROSURI();
      RosMainNode rosMainNode = new RosMainNode(masterURI, "ImagePublisher", true);
      rosMainNode.attachSubscriber("/depthcam/color/image_raw/compressed", this);
      rosMainNode.execute();

      LogTools.info(ROS2Tools.D435_VIDEO);
      imagePublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.D435_VIDEO);
   }

   int i = 0;

   @Override
   public void onNewMessage(sensor_msgs.CompressedImage ros1Image)
   {
      try
      {
//         LogTools.info(StringTools.format("Format: {}", ros1Image.getFormat()));
         CompressedImage ros2Image = new CompressedImage();
         //      ros2Image.setHeight(ros1Image.getHeight());
         //      ros2Image.setWidth(ros1Image.getWidth());
         byte[] data = ros1Image.getData().array();
         int dataOffset = ros1Image.getData().arrayOffset();
         int length = data.length;
                  ros2Image.getData().add(data, dataOffset, length - dataOffset);
         //         ros2Image.getData().add(data);

         //      Image ros2Image = new Image();
         //      ros2Image.setHeight(ros1Image.getHeight());
         //      ros2Image.setWidth(ros1Image.getWidth());
         //      byte[] data = ros1Image.getData().array();
         //      int dataOffset = ros1Image.getData().arrayOffset();
         //      int length = data.length;
         ////      LogTools.info(StringTools.format("Copying width {} height {} length {} offset: {}",
         ////                                       ros1Image.getWidth(), ros1Image.getHeight(), length, dataOffset));
         ////      ros2Image.getData().add(data, dataOffset, length - dataOffset);
         //      ros2Image.getData().add(data);

         //      Image32 ros2Image = new Image32();
         //      ros2Image.setHeight(ros1Image.getHeight());
         //      ros2Image.setWidth(ros1Image.getWidth());
         //
         //      ChannelBuffer data = ros1Image.getData();
         //      byte[] array = data.array();
         //      int dataIndex = data.arrayOffset();
         //      for (int i = 0; i < ros1Image.getHeight(); i++)
         //      {
         //         for (int j = 0; j < ros1Image.getWidth(); j++)
         //         {
         //            int b = array[dataIndex];
         //            dataIndex++;
         //            int g = array[dataIndex];
         //            dataIndex++;
         //            int r = array[dataIndex];
         //            dataIndex++;
         //
         //            int rgbColor = convertBGR2RGB(b, g, r);
         //            ros2Image.getRgbdata().add(rgbColor);
         //         }
         //      }

         //      ChannelBuffer data = ros1Image.getData();
         //      byte[] array = data.array();
         //      int dataIndex = data.arrayOffset();
         //      for (int i = 0; i < ros1Image.getHeight(); i++)
         //      {
         //         for (int j = 0; j < ros1Image.getWidth(); j++)
         //         {
         //            byte b = array[dataIndex];
         //            dataIndex++;
         //            byte g = array[dataIndex];
         //            dataIndex++;
         //            byte r = array[dataIndex];
         //            dataIndex++;
         //
         //            ros2Image.getData().add(r);
         //            ros2Image.getData().add(g);
         //            ros2Image.getData().add(b);
         //         }
         //      }

         imagePublisher.publish(ros2Image);
      }
      catch (Exception e)
      {
         LogTools.error(e.getMessage());
         e.printStackTrace();
      }
   }

   private static int convertBGR2RGB(int b, int g, int r)
   {
      int rgb = ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | (b & 0xFF);
      return rgb;
   }

   public static void main(String[] args) throws URISyntaxException, IOException
   {
      new RealsenseImageROS1Bridge();
   }
}
