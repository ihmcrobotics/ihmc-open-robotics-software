package us.ihmc.perception.streaming;

import org.apache.logging.log4j.core.util.ExecutorServices;
import perception_msgs.msg.dds.SRTStreamStatus;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.perception.RawImage;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

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
                         int inputAVPixelFormat)
   {
      addStream(streamTopic, exampleImage, inputAVPixelFormat, -1, false);
   }

   public void addStream(ROS2Topic<SRTStreamStatus> streamTopic,
                         RawImage exampleImage,
                         int inputAVPixelFormat,
                         int intermediateColorConversion,
                         boolean useHardwareAcceleration)
   {
      ROS2SRTVideoStreamer videoStreamer = new ROS2SRTVideoStreamer(ros2Node, streamTopic);
      videoStreamer.initialize(exampleImage, inputAVPixelFormat, intermediateColorConversion, useHardwareAcceleration);
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
