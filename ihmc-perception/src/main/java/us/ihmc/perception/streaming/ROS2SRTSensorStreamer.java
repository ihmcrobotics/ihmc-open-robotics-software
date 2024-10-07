package us.ihmc.perception.streaming;

import org.apache.logging.log4j.core.util.ExecutorServices;
import perception_msgs.msg.dds.SRTStreamStatus;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import static org.bytedeco.ffmpeg.global.avutil.*;
import static org.bytedeco.opencv.global.opencv_imgproc.COLOR_BGR2BGRA;
import static org.bytedeco.opencv.global.opencv_imgproc.COLOR_RGB2RGBA;

public class ROS2SRTSensorStreamer
{
   private final ROS2Node ros2Node;

   private final Map<ROS2Topic<SRTStreamStatus>, ROS2SRTVideoStreamer> videoStreamers = new HashMap<>();
   private final ExecutorService sendFrameExecutor = Executors.newCachedThreadPool();

   public ROS2SRTSensorStreamer()
   {
      ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, getClass().getSimpleName().toLowerCase() + "_node");
   }

   public void addStream(ROS2Topic<SRTStreamStatus> streamTopic,
                         RawImage exampleImage,
                         int inputAVPixelFormat,
                         int intermediateColorConversion,
                         boolean useHardwareAcceleration)
   {
      ROS2SRTVideoStreamer videoStreamer = new ROS2SRTVideoStreamer(ros2Node, streamTopic);
      if (inputAVPixelFormat == AV_PIX_FMT_GRAY16)
         videoStreamer.initializeForDepth(exampleImage);
      else
         videoStreamer.initializeForColor(exampleImage, inputAVPixelFormat, intermediateColorConversion, useHardwareAcceleration);
      videoStreamers.put(streamTopic, videoStreamer);
   }

   public boolean hasStream(ROS2Topic<SRTStreamStatus> streamTopic)
   {
      return videoStreamers.containsKey(streamTopic);
   }

   public void sendFrame(ROS2Topic<SRTStreamStatus> streamTopic, RawImage frame)
   {
      if (frame.get() == null)
         return;

      if (!hasStream(streamTopic))
         addStreamWithGuessedParameters(streamTopic, frame);

      sendFrameExecutor.submit(() ->
      {
         videoStreamers.get(streamTopic).sendFrame(frame);
         frame.release();
      });
   }

   public void destroy()
   {
      ExecutorServices.shutdown(sendFrameExecutor, 2, TimeUnit.SECONDS, getClass().getSimpleName());

      for (ROS2SRTVideoStreamer videoStreamer : videoStreamers.values())
         videoStreamer.destroy();
   }

   private void addStreamWithGuessedParameters(ROS2Topic<SRTStreamStatus> streamTopic, RawImage exampleImage)
   {
      switch (exampleImage.getPixelFormat())
      {
         case GRAY16 -> addStream(streamTopic, exampleImage, exampleImage.getPixelFormat().toFFmpegPixelFormat(), -1, false);
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
