package us.ihmc.perception.streaming;

import org.apache.logging.log4j.core.util.ExecutorServices;
import perception_msgs.msg.dds.SRTStreamMessage;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.perception.RawImage;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

public class ROS2SRTSensorStreamer
{
   private final Map<ROS2IOTopicPair<SRTStreamMessage>, ROS2SRTVideoStreamer> videoStreamers = new HashMap<>();
   private final ExecutorService sendFrameExecutor = Executors.newCachedThreadPool();

   public ROS2SRTSensorStreamer()
   {
      // Just here so it can be searched for
   }

   public void addStream(ROS2IOTopicPair<SRTStreamMessage> streamTopic, RawImage exampleImage, double inputFPS, int inputAVPixelFormat)
   {
      ROS2SRTVideoStreamer videoStreamer = new ROS2SRTVideoStreamer(streamTopic);
      videoStreamer.initialize(exampleImage, inputFPS, inputAVPixelFormat);
      videoStreamers.put(streamTopic, videoStreamer);
   }

   public boolean hasStream(ROS2IOTopicPair<SRTStreamMessage> streamTopic)
   {
      return videoStreamers.containsKey(streamTopic);
   }

   public void sendFrame(ROS2IOTopicPair<SRTStreamMessage> streamTopic, RawImage frame)
   {
      if (frame.get() == null)
         return;

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
}
