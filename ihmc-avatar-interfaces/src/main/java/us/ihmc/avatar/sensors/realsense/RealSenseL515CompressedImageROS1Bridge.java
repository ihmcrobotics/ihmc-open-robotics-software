package us.ihmc.avatar.sensors.realsense;

import controller_msgs.msg.dds.VideoPacket;
import sensor_msgs.CompressedImage;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.yuv.JPEGEncoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.SingleThreadSizeOneQueueExecutor;
import us.ihmc.tools.Timer;
import us.ihmc.tools.UnitConversions;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RealSenseCompressedImageSubscriber;
import us.ihmc.utilities.ros.subscriber.RosCameraInfoSubscriber;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.net.URI;
import java.net.URISyntaxException;
import java.nio.ByteBuffer;

public class RealSenseL515CompressedImageROS1Bridge
{
   private static final double MIN_PUBLISH_PERIOD = UnitConversions.hertzToSeconds(4);

   private String l515CompressedImageTopic = "/camera/color/image_raw/compressed";
   private String l515CameraInfoTopic = "/camera/color/camera_info";

   private BufferedImage latestBufferedImage;
   private final IHMCROS2Publisher<VideoPacket> videoPacketPublisher;

   private final Timer throttleTimer = new Timer();
   private static final JPEGEncoder encoder = new JPEGEncoder();
   private static final YUVPictureConverter converter = new YUVPictureConverter();
   private final SingleThreadSizeOneQueueExecutor executor = new SingleThreadSizeOneQueueExecutor(getClass().getSimpleName());

   private RosCameraInfoSubscriber colorImageInfoSubscriber;

   private final IHMCROS2Publisher<VideoPacket> colorPublisher;

   public RealSenseL515CompressedImageROS1Bridge(RosMainNode rosNode, ROS2Node ros2Node)
   {
      ROS2Topic<VideoPacket> ros2Topic = ROS2Tools.VIDEO;
      videoPacketPublisher = ROS2Tools.createPublisher(ros2Node, ros2Topic, ROS2QosProfile.DEFAULT());

      new RealSenseCompressedImageSubscriber(rosNode, l515CompressedImageTopic)
      {
         @Override
         protected void compressedImageReceived(CompressedImage image)
         {
            executor.submitTask(() -> repackAndPublish(image));
         }
      };

      rosNode.execute();

      colorPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, VideoPacket.class, ROS2Tools.IHMC_ROOT);
      colorImageInfoSubscriber = new RosCameraInfoSubscriber(rosNode, l515CameraInfoTopic);
      rosNode.attachSubscriber(l515CameraInfoTopic, colorImageInfoSubscriber);
   }

   private void repackAndPublish(CompressedImage image)
   {
      throttleTimer.sleepUntilExpiration(MIN_PUBLISH_PERIOD);
      throttleTimer.reset();

      LogTools.info("Message Received: " + image);

      try
      {
         byte[] payload = image.getData().array();
         int offset = image.getData().arrayOffset();
         BufferedImage bufferedImage = ImageIO.read(new ByteArrayInputStream(payload, offset, payload.length - offset));

         YUVPicture picture = converter.fromBufferedImage(bufferedImage, YUVPicture.YUVSubsamplingType.YUV420);
         ByteBuffer buffer = encoder.encode(picture, 75);

         byte[] data = new byte[buffer.remaining()];
         buffer.get(data);

         VideoPacket message = new VideoPacket();
         message.setTimestamp(System.nanoTime());
         message.getData().add(data);

         videoPacketPublisher.publish(message);
      }
      catch (Exception e)
      {
         LogTools.error(e.getMessage());
         e.printStackTrace();
      }
   }

   public static void main(String[] args) throws URISyntaxException
   {
      RosMainNode rosMainNode = new RosMainNode(new URI("http://localhost:11311"), "RealSenseL515DataPublisher", true);
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "video_viewer");
      new RealSenseL515CompressedImageROS1Bridge(rosMainNode, ros2Node);
   }
}
