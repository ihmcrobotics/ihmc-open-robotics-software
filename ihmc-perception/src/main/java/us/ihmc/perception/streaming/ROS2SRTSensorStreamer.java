package us.ihmc.perception.streaming;

import perception_msgs.SRTStreamStatus;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Topic;

import java.util.HashMap;
import java.util.Map;

import static org.bytedeco.ffmpeg.global.avutil.*;
import static org.bytedeco.opencv.global.opencv_imgproc.COLOR_BGR2BGRA;
import static org.bytedeco.opencv.global.opencv_imgproc.COLOR_RGB2RGBA;

public class ROS2SRTSensorStreamer
{
   private final ROS2Node ros2Node;
   private boolean destroyROS2Node = false;

   private final Map<ROS2Topic<SRTStreamStatus>, ROS2SRTVideoStreamer> videoStreamers = new HashMap<>();

   public ROS2SRTSensorStreamer()
   {
      this(new ROS2Node(ROS2SRTSensorStreamer.class.getSimpleName().toLowerCase() + "_node"));
      destroyROS2Node = true;
   }

   public ROS2SRTSensorStreamer(ROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;
   }

   public void addStream(ROS2Topic<SRTStreamStatus> streamTopic,
                         RawImage exampleImage,
                         int inputAVPixelFormat,
                         int intermediateColorConversion,
                         boolean useHardwareAcceleration)
   {
      addStream(streamTopic, exampleImage, inputAVPixelFormat, intermediateColorConversion, useHardwareAcceleration, false);
   }

   public void addStream(ROS2Topic<SRTStreamStatus> streamTopic,
                         RawImage exampleImage,
                         int inputAVPixelFormat,
                         int intermediateColorConversion,
                         boolean useHardwareAcceleration,
                         boolean highQuality)
   {
      ROS2SRTVideoStreamer videoStreamer = new ROS2SRTVideoStreamer(ros2Node, streamTopic);
      if (inputAVPixelFormat == AV_PIX_FMT_GRAY16)
         videoStreamer.initializeForDepth(exampleImage);
      else
         videoStreamer.initializeForColor(exampleImage, inputAVPixelFormat, intermediateColorConversion, useHardwareAcceleration, highQuality);
      videoStreamers.put(streamTopic, videoStreamer);
   }

   public boolean hasStream(ROS2Topic<SRTStreamStatus> streamTopic)
   {
      return videoStreamers.containsKey(streamTopic);
   }

   public void sendFrame(ROS2Topic<SRTStreamStatus> streamTopic, RawImage frame)
   {
      if (frame == null || frame.get() == null)
         return;

      if (!hasStream(streamTopic))
         addStreamWithGuessedParameters(streamTopic, frame);

      videoStreamers.get(streamTopic).sendFrame(frame);
      frame.release();
   }

   public void destroy()
   {
      for (ROS2SRTVideoStreamer videoStreamer : videoStreamers.values())
         videoStreamer.destroy();

      if (destroyROS2Node)
         ros2Node.close();
   }

   private void addStreamWithGuessedParameters(ROS2Topic<SRTStreamStatus> streamTopic, RawImage exampleImage)
   {
      switch (exampleImage.getPixelFormat())
      {
         case GRAY16 -> addStream(streamTopic, exampleImage, exampleImage.getPixelFormat().toFFmpegPixelFormat(), -1, false);
         case YUV444P -> addStream(streamTopic, exampleImage, AV_PIX_FMT_YUV444P, -1, true, true); // Settings for colorized depth
         case BGR8 -> addStream(streamTopic, exampleImage, AV_PIX_FMT_BGR0, COLOR_BGR2BGRA, true);
         case BGRA8 -> addStream(streamTopic, exampleImage, AV_PIX_FMT_BGR0, -1, true); // This will lose the alpha channel
         case RGB8 -> addStream(streamTopic, exampleImage, AV_PIX_FMT_RGB0, COLOR_RGB2RGBA, true);
         case RGBA8 -> addStream(streamTopic, exampleImage, AV_PIX_FMT_RGB0, -1, true); // This will lose the alpha channel
         default ->
         {
            LogTools.warn("The best streaming configuration for {} is unknown. Calling addStream() with proper parameters may result in better performance.",
                          streamTopic.getName());
            addStream(streamTopic, exampleImage, exampleImage.getPixelFormat().toFFmpegPixelFormat(), -1, false);
         }
      }
   }
}
