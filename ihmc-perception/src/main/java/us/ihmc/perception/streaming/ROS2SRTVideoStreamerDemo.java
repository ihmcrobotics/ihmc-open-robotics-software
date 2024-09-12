package us.ihmc.perception.streaming;

import org.bytedeco.ffmpeg.global.avutil;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.perception.realsense.RealsenseDeviceManager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.RealsenseColorDepthImageRetriever;
import us.ihmc.sensors.ZEDColorDepthImageRetrieverSVO;
import us.ihmc.sensors.ZEDColorDepthImageRetrieverSVO.RecordMode;
import us.ihmc.tools.IHMCCommonPaths;

public class ROS2SRTVideoStreamerDemo
{
   private static final String SVO_FILE_NAME = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20240715_103234_ZEDRecording_NewONRCourseWalk.svo2")
                                                                                        .toAbsolutePath()
                                                                                        .toString();

   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "srt_video_streamer_demo");
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

   private final ROS2SRTVideoStreamer streamer = new ROS2SRTVideoStreamer(PerceptionAPI.REALSENSE_COLOR_STREAM);
   private final ZEDColorDepthImageRetrieverSVO imageRetriever;

   private volatile boolean running = true;
   private final Notification doneNotification = new Notification();

   private ROS2SRTVideoStreamerDemo()
   {
      imageRetriever = new ZEDColorDepthImageRetrieverSVO(0, () -> true, () -> true, ros2Helper, RecordMode.PLAYBACK, SVO_FILE_NAME);
      imageRetriever.start();

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, getClass().getSimpleName() + "Destruction"));

      while (running)
      {
         RawImage colorImage = imageRetriever.getLatestRawColorImage(RobotSide.LEFT);
         if (!streamer.isInitialized())
            streamer.initialize(colorImage, 20.0, avutil.AV_PIX_FMT_BGR24);
         streamer.sendFrame(colorImage);
         colorImage.release();
      }

      doneNotification.set();
   }

   private void destroy()
   {
      running = false;

      doneNotification.blockingPoll();

      streamer.destroy();
      imageRetriever.destroy();
      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      new ROS2SRTVideoStreamerDemo();
   }
}
