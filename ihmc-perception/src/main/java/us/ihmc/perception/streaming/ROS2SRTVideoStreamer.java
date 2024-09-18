package us.ihmc.perception.streaming;

import perception_msgs.msg.dds.SRTStreamStatus;
import us.ihmc.perception.RawImage;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.ros2.ROS2Topic;

import java.net.InetSocketAddress;

public class ROS2SRTVideoStreamer
{
   private final SRTStreamStatus statusMessage;
   private final ROS2PublisherBasics<SRTStreamStatus> statusMessagePublisher;

   private final SRTVideoStreamer videoStreamer;

   public ROS2SRTVideoStreamer(ROS2Node ros2Node, ROS2Topic<SRTStreamStatus> streamTopic)
   {
      this(ros2Node, streamTopic, StreamingTools.getMyAddress());
   }

   public ROS2SRTVideoStreamer(ROS2Node ros2Node, ROS2Topic<SRTStreamStatus> streamTopic, InetSocketAddress streamOutputAddress)
   {
      statusMessage = new SRTStreamStatus();
      statusMessage.setStreamerAddress(streamOutputAddress.getHostString());
      statusMessage.setStreamerPort(streamOutputAddress.getPort());

      statusMessagePublisher = ros2Node.createPublisher(streamTopic);

      videoStreamer = new SRTVideoStreamer(streamOutputAddress);
   }

   public void initialize(RawImage exampleImage, double inputFPS, int inputPixelFormat)
   {
      initialize(exampleImage.getImageWidth(), exampleImage.getImageHeight(), inputFPS, inputPixelFormat);
   }

   public void initialize(int imageWidth, int imageHeight, double inputFPS, int inputPixelFormat)
   {
      videoStreamer.initialize(imageWidth, imageHeight, inputFPS, inputPixelFormat);
   }

   public synchronized void sendFrame(RawImage frame)
   {
      if (frame.get() == null)
         return;

      videoStreamer.sendFrame(frame.getCpuImageMat());

      statusMessage.setIsStreaming(true);
      statusMessage.setImageWidth(frame.getImageWidth());
      statusMessage.setImageHeight(frame.getImageHeight());
      statusMessage.setFx(frame.getFocalLengthX());
      statusMessage.setFy(frame.getFocalLengthY());
      statusMessage.setCx(frame.getPrincipalPointX());
      statusMessage.setCy(frame.getPrincipalPointY());
      statusMessage.setDepthDiscretization(frame.getDepthDiscretization());
      statusMessage.getPosition().set(frame.getPosition());
      statusMessage.getOrientation().set(frame.getOrientation());
      statusMessagePublisher.publish(statusMessage);

      frame.release();
   }

   public synchronized void destroy()
   {
      statusMessage.setIsStreaming(false);
      statusMessagePublisher.publish(statusMessage);

      videoStreamer.destroy();
      statusMessagePublisher.remove();
   }

   public int connectedCallerCount()
   {
      return videoStreamer.connectedCallerCount();
   }

   public boolean isInitialized()
   {
      return videoStreamer.isInitialized();
   }
}
