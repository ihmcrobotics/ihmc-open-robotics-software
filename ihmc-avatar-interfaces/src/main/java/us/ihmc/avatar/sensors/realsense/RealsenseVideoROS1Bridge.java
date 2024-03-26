package us.ihmc.avatar.sensors.realsense;

import perception_msgs.msg.dds.VideoPacket;
import sensor_msgs.msg.dds.CompressedImage;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.yuv.JPEGEncoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.Timer;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.nio.ByteBuffer;

public class RealsenseVideoROS1Bridge extends AbstractRosTopicSubscriber<sensor_msgs.CompressedImage>
{
   private static final boolean THROTTLE = true;
   private final double outputFrequenct;

   private final IHMCROS2Publisher<VideoPacket> publisher;
   private final Timer throttleTimer = new Timer();
   private final ResettableExceptionHandlingExecutorService executor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

   private final YUVPictureConverter converter = new YUVPictureConverter();
   private final JPEGEncoder encoder = new JPEGEncoder();

   public RealsenseVideoROS1Bridge(RosMainNode ros1Node,
                                   ROS2Node ros2Node,
                                   String ros1InputTopic,
                                   ROS2Topic<VideoPacket> ros2OutputTopic,
                                   double outputFrequency)
   {
      super(sensor_msgs.CompressedImage._TYPE);
      outputFrequenct = UnitConversions.hertzToSeconds(outputFrequency);

      String ros1Topic = ros1InputTopic;
      LogTools.info("Subscribing ROS 1: {}", ros1Topic);
      ros1Node.attachSubscriber(ros1Topic, this);

      ROS2Topic<VideoPacket> ros2Topic = ros2OutputTopic;
      LogTools.info("Publishing ROS 2: {}", ros2Topic.getName());
      publisher = ROS2Tools.createPublisher(ros2Node, ros2Topic);
   }

   @Override
   public void onNewMessage(sensor_msgs.CompressedImage ros1Image)
   {
      if (THROTTLE)
      {
         executor.clearQueueAndExecute(() -> waitThenAct(ros1Image));
      }
      else
      {
         compute(ros1Image);
      }
   }

   private void waitThenAct(sensor_msgs.CompressedImage ros1Image)
   {
      throttleTimer.sleepUntilExpiration(outputFrequenct);
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
