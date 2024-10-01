package us.ihmc.perception.streaming;

import org.bytedeco.javacpp.BytePointer;
import perception_msgs.msg.dds.SRTStreamStatus;
import perception_msgs.msg.dds.StreamData;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.log.LogTools;
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
   private final StreamData frameDataMessage;

   private final SRTVideoStreamer videoStreamer;

   private final FrequencyCalculator sendFrequencyCalculator;

   public ROS2SRTVideoStreamer(ROS2Node ros2Node, ROS2Topic<SRTStreamStatus> streamTopic)
   {
      this(ros2Node, streamTopic, StreamingTools.getHostAddress());
   }

   public ROS2SRTVideoStreamer(ROS2Node ros2Node, ROS2Topic<SRTStreamStatus> streamTopic, InetSocketAddress streamOutputAddress)
   {
      LogTools.info("Streaming {} on {}", streamTopic.getName(), streamOutputAddress);

      statusMessage = new SRTStreamStatus();
      statusMessage.setStreamerAddress(streamOutputAddress.getHostString());
      statusMessage.setStreamerPort(streamOutputAddress.getPort());

      statusMessagePublisher = ros2Node.createPublisher(streamTopic);

      frameDataMessage = new StreamData();

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
      videoStreamer.initialize(imageWidth, imageHeight, inputPixelFormat, intermediateColorConversion, streamLosslessly, true, useHardwareAcceleration);
   }

   public synchronized void sendFrame(RawImage frame)
   {
      if (frame.get() == null)
         return;

      frameDataMessage.setImageWidth(frame.getImageWidth());
      frameDataMessage.setImageHeight(frame.getImageHeight());
      frameDataMessage.setFx(frame.getFocalLengthX());
      frameDataMessage.setFy(frame.getFocalLengthY());
      frameDataMessage.setCx(frame.getPrincipalPointX());
      frameDataMessage.setCy(frame.getPrincipalPointY());
      frameDataMessage.setDepthDiscretization(frame.getDepthDiscretization());
      frameDataMessage.getPosition().set(frame.getPosition());
      frameDataMessage.getOrientation().set(frame.getOrientation());
      try (BytePointer serializedMessage = new BytePointer(MessageTools.serialize(frameDataMessage)))
      {
         videoStreamer.sendFrame(frame, serializedMessage);
      }

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
