package us.ihmc.perception.streaming;

import perception_msgs.msg.dds.SRTStreamStatus;
import us.ihmc.perception.RawImage;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.time.FrequencyCalculator;

import java.net.InetSocketAddress;

public class ROS2SRTVideoStreamer
{
   private final SRTStreamStatus statusMessage;
   private final ROS2PublisherBasics<SRTStreamStatus> statusMessagePublisher;

   private final SRTVideoStreamer videoStreamer;

   private final FrequencyCalculator sendFrequencyCalculator;

   public ROS2SRTVideoStreamer(ROS2Node ros2Node, ROS2Topic<SRTStreamStatus> streamTopic)
   {
      this(ros2Node, streamTopic, StreamingTools.getHostAddress());
   }

   public ROS2SRTVideoStreamer(ROS2Node ros2Node, ROS2Topic<SRTStreamStatus> streamTopic, InetSocketAddress streamOutputAddress)
   {
      statusMessage = new SRTStreamStatus();
      statusMessage.setStreamerAddress(streamOutputAddress.getHostString());
      statusMessage.setStreamerPort(streamOutputAddress.getPort());

      statusMessagePublisher = ros2Node.createPublisher(streamTopic);

      videoStreamer = new SRTVideoStreamer(streamOutputAddress);

      sendFrequencyCalculator = new FrequencyCalculator();
   }

   public void initialize(RawImage exampleImage, int inputPixelFormat)
   {
      initialize(exampleImage.getImageWidth(), exampleImage.getImageHeight(), inputPixelFormat);
   }

   public void initialize(int imageWidth,
                          int imageHeight,
                          int inputPixelFormat)
   {
      initialize(imageWidth, imageHeight, inputPixelFormat, -1, false, false);
   }

   public void initialize(RawImage exampleImage,
                          int inputPixelFormat,
                          int intermediateColorConversion,
                          boolean streamLosslessly,
                          boolean useHardwareAcceleration)
   {
      initialize(exampleImage.getImageWidth(),
                 exampleImage.getImageHeight(),
                 inputPixelFormat,
                 intermediateColorConversion,
                 streamLosslessly,
                 useHardwareAcceleration);
   }

   public void initialize(int imageWidth,
                          int imageHeight,
                          int inputPixelFormat,
                          int intermediateColorConversion,
                          boolean streamLosslessly,
                          boolean useHardwareAcceleration)
   {
      videoStreamer.initialize(imageWidth, imageHeight, inputPixelFormat, intermediateColorConversion, streamLosslessly, useHardwareAcceleration);
   }

   public synchronized void sendFrame(RawImage frame)
   {
      if (frame.get() == null)
         return;

      if (videoStreamer.isUsingHardwareAcceleration())
         videoStreamer.sendFrame(frame.getGpuImageMat());
      else
         videoStreamer.sendFrame(frame.getCpuImageMat());

      sendFrequencyCalculator.ping();
      float frequency = (float) sendFrequencyCalculator.getFrequency();
      statusMessage.setExpectedPublishFrequency(Math.max(1.0f, frequency));
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
