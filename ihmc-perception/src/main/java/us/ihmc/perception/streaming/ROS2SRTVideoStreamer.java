package us.ihmc.perception.streaming;

import static org.bytedeco.ffmpeg.global.avutil.*;

import org.bytedeco.javacpp.BytePointer;
import perception_msgs.SRTStreamStatus;
import perception_msgs.VideoFrameExtraData;
import us.ihmc.commons.time.FrequencyCalculator;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.fastddsjava.cdr.CDRBuffer;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;

import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.util.Map;

public class ROS2SRTVideoStreamer
{
   private static final String PREFERRED_COLOR_CODEC = "hevc_nvenc";
   private static final String COLOR_OUTPUT_FORMAT = "mpegts";
   private static final String PREFERRED_DEPTH_CODEC = "ffv1";
   private static final String DEPTH_OUTPUT_FORMAT = "matroska";
   private static final int COLOR_OUTPUT_PIXEL_FORMAT = AV_PIX_FMT_YUV444P;

   private final ROS2Node ros2Node;
   private final SRTStreamStatus statusMessage;
   private final ROS2Publisher<SRTStreamStatus> statusMessagePublisher;
   private final VideoFrameExtraData frameExtraData;

   private final SRTVideoStreamer videoStreamer;

   private boolean isStreamingDepth;

   private final FrequencyCalculator sendFrequencyCalculator;

   public ROS2SRTVideoStreamer(ROS2Node ros2Node, ROS2Topic<SRTStreamStatus> streamTopic)
   {
      this(ros2Node, streamTopic, StreamingTools.getHostAddress());
   }

   public ROS2SRTVideoStreamer(ROS2Node ros2Node, ROS2Topic<SRTStreamStatus> streamTopic, InetSocketAddress streamOutputAddress)
   {
      av_log_set_level(AV_LOG_FATAL); // silences non monotonically increasing dts warning, which are 99% safe to ignore

      LogTools.info("Streaming {} on {}", streamTopic.getName(), streamOutputAddress);

      this.ros2Node = ros2Node;
      statusMessage = new SRTStreamStatus();
      statusMessage.setStreamerAddress(streamOutputAddress.getHostString());
      statusMessage.setStreamerPort(streamOutputAddress.getPort());

      statusMessagePublisher = ros2Node.createPublisher(streamTopic);

      frameExtraData = new VideoFrameExtraData();

      videoStreamer = new SRTVideoStreamer(streamOutputAddress);

      sendFrequencyCalculator = new FrequencyCalculator();
   }

   public void initializeForColor(RawImage exampleImage, int inputPixelFormat)
   {
      initializeForColor(exampleImage.getWidth(), exampleImage.getHeight(), inputPixelFormat);
   }

   public void initializeForColor(int imageWidth, int imageHeight, int inputPixelFormat)
   {
      initializeForColor(imageWidth, imageHeight, inputPixelFormat, -1, false);
   }

   public void initializeForColor(RawImage exampleImage, int inputPixelFormat, int intermediateColorConversion, boolean useHardwareAcceleration)
   {
      initializeForColor(exampleImage.getWidth(), exampleImage.getHeight(), inputPixelFormat, intermediateColorConversion, useHardwareAcceleration);
   }

   public void initializeForColor(int imageWidth, int imageHeight, int inputPixelFormat, int intermediateColorConversion, boolean useHardwareAcceleration)
   {
      initializeForColor(imageWidth, imageHeight, inputPixelFormat, intermediateColorConversion, useHardwareAcceleration, false);
   }

   public void initializeForColor(RawImage exampleImage,
                                  int inputPixelFormat,
                                  int intermediateColorConversion,
                                  boolean useHardwareAcceleration,
                                  boolean highQuality)
   {
      initializeForColor(exampleImage.getWidth(),
                         exampleImage.getHeight(),
                         inputPixelFormat,
                         intermediateColorConversion,
                         useHardwareAcceleration,
                         highQuality);
   }

   public void initializeForColor(int imageWidth,
                                  int imageHeight,
                                  int inputPixelFormat,
                                  int intermediateColorConversion,
                                  boolean useHardwareAcceleration,
                                  boolean highQuality)
   {
      Map<String, String> hevcOptions = highQuality ? StreamingTools.getHEVCNVENCHighQualityOptions() : StreamingTools.getHEVCNVENCStreamingOptions();
      hevcOptions.put("udu_sei", "1");
      videoStreamer.initialize(imageWidth,
                               imageHeight,
                               inputPixelFormat,
                               COLOR_OUTPUT_PIXEL_FORMAT,
                               intermediateColorConversion,
                               COLOR_OUTPUT_FORMAT,
                               PREFERRED_COLOR_CODEC,
                               hevcOptions,
                               useHardwareAcceleration);
      isStreamingDepth = false;
   }

   public void initializeForDepth(RawImage exampleImage)
   {
      initializeForDepth(exampleImage.getWidth(), exampleImage.getHeight());
   }

   public void initializeForDepth(int imageWidth, int imageHeight)
   {
      Map<String, String> ffv1Options = StreamingTools.getFFV1StreamingOptions();
      videoStreamer.initialize(imageWidth,
                               imageHeight,
                               AV_PIX_FMT_GRAY16,
                               AV_PIX_FMT_GRAY16,
                               -1,
                               DEPTH_OUTPUT_FORMAT,
                               PREFERRED_DEPTH_CODEC,
                               ffv1Options,
                               false);
      isStreamingDepth = true;
   }

   public synchronized void sendFrame(RawImage frame)
   {
      if (frame.get() == null)
         return;

      frameExtraData.setSequenceNumber((int) frame.getSequenceNumber());
      MessageTools.toMessage(frame.getAcquisitionTime(), frameExtraData.getAcquisitionTime());
      frameExtraData.getSensorPose().set(frame.getTransformToWorld());

      if (isStreamingDepth)
      {
         videoStreamer.sendFrame(frame);
         statusMessage.getFrameExtraData().set(frameExtraData);
         statusMessage.setContainsExtraData(true);
      }
      else
      {
         CDRBuffer cdrBuffer = new CDRBuffer();
         cdrBuffer.ensureRemainingCapacity(frameExtraData.calculateSizeBytes(0) + CDRBuffer.PAYLOAD_HEADER.length);
         cdrBuffer.writePayloadHeader();
         frameExtraData.serialize(cdrBuffer);
         ByteBuffer underlyingBuffer = cdrBuffer.getBufferUnsafe();
         underlyingBuffer.flip();
         BytePointer serializedMessage = new BytePointer(underlyingBuffer);
         videoStreamer.sendFrame(frame, serializedMessage);
         serializedMessage.close();
         statusMessage.setContainsExtraData(false);
      }

      sendFrequencyCalculator.ping();
      float frequency = (float) sendFrequencyCalculator.getFrequency();
      statusMessage.setExpectedPublishFrequency(Math.max(1.0f, frequency));
      statusMessage.setIsStreaming(true);
      statusMessage.setImageWidth((short) frame.getWidth());
      statusMessage.setImageHeight((short) frame.getHeight());
      statusMessage.setFx(frame.getFocalLengthX());
      statusMessage.setFy(frame.getFocalLengthY());
      statusMessage.setCx(frame.getPrincipalPointX());
      statusMessage.setCy(frame.getPrincipalPointY());
      statusMessage.setCameraModel(frame.getCameraModel().toByte());
      statusMessage.setDepthDiscretization(frame.getDepthDiscretization());
      statusMessagePublisher.publish(statusMessage);

      frame.release();
   }

   public synchronized void destroy()
   {
      statusMessage.setIsStreaming(false);
      statusMessagePublisher.publish(statusMessage);

      videoStreamer.destroy();
      ros2Node.destroyPublisher(statusMessagePublisher);
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
