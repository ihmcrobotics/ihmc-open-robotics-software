package us.ihmc.avatar.sensors.realsense;

import controller_msgs.msg.dds.VideoPacket;
import sensor_msgs.msg.dds.CompressedImage;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.yuv.JPEGEncoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.SingleThreadSizeOneQueueExecutor;
import us.ihmc.tools.Timer;
import us.ihmc.tools.UnitConversions;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.nio.ByteBuffer;

public class RealsenseD435VideoROS1Bridge extends AbstractRosTopicSubscriber<sensor_msgs.CompressedImage>
{
   private static final boolean THROTTLE = false;
   private static final double MIN_PUBLISH_PERIOD = UnitConversions.hertzToSeconds(24.0);

   private final IHMCROS2Publisher<VideoPacket> publisher;
   private final Timer throttleTimer = new Timer();
   private final SingleThreadSizeOneQueueExecutor executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

   private final YUVPictureConverter converter = new YUVPictureConverter();
   private final JPEGEncoder encoder = new JPEGEncoder();

   public RealsenseD435VideoROS1Bridge(RosMainNode ros1Node, ROS2Node ros2Node)
   {
      super(sensor_msgs.CompressedImage._TYPE);

      String ros1Topic = RosTools.D435_VIDEO;
      LogTools.info("Subscribing ROS 1: {}", ros1Topic);
      ros1Node.attachSubscriber(ros1Topic, this);

      ROS2Topic<VideoPacket> ros2Topic = ROS2Tools.D435_VIDEO;
      LogTools.info("Publishing ROS 2: {}", ros2Topic.getName());
      publisher = ROS2Tools.createPublisher(ros2Node, ros2Topic, ROS2QosProfile.DEFAULT());
   }

   @Override
   public void onNewMessage(sensor_msgs.CompressedImage ros1Image)
   {
      if (THROTTLE)
      {
         executor.submitTask(() -> waitThenAct(ros1Image));
      }
      else
      {
         compute(ros1Image);
      }
   }

   private void waitThenAct(sensor_msgs.CompressedImage ros1Image)
   {
      throttleTimer.sleepUntilExpiration(MIN_PUBLISH_PERIOD);
      throttleTimer.reset();

      compute(ros1Image);
   }

   private void compute(sensor_msgs.CompressedImage ros1Image)
   {
      try
      {
         byte[] payload = ros1Image.getData().array();
         int offset = ros1Image.getData().arrayOffset();
         BufferedImage bufferedImage = ImageIO.read(new ByteArrayInputStream(payload, offset, payload.length - offset));

         YUVPicture picture = converter.fromBufferedImage(bufferedImage, YUVPicture.YUVSubsamplingType.YUV420);
         ByteBuffer buffer = encoder.encode(picture, 75);

         byte[] data = new byte[buffer.remaining()];
         buffer.get(data);

         VideoPacket message = new VideoPacket();
         message.setTimestamp(System.nanoTime());
         message.getData().add(data);

         publisher.publish(message);
      }
      catch (Exception e)
      {
         LogTools.error(e.getMessage());
         e.printStackTrace();
      }
   }

   // Eventually it would be nice to use Compressed Image
   private void createROS2CompressedImage(sensor_msgs.CompressedImage ros1Image)
   {
      CompressedImage ros2Image = new CompressedImage();
      byte[] data = ros1Image.getData().array();
      int dataOffset = ros1Image.getData().arrayOffset();
      int length = data.length;
      ros2Image.getData().add(data, dataOffset, length - dataOffset);
   }
}
