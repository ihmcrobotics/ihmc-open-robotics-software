package us.ihmc.perception.streaming;

import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.opencv.global.opencv_imgproc;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.RawImage;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;
import us.ihmc.sensors.ZEDColorDepthImageRetrieverSVO;
import us.ihmc.sensors.ZEDColorDepthImageRetrieverSVO.RecordMode;
import us.ihmc.tools.IHMCCommonPaths;

public class ROS2SRTVideoStreamingDemo
{
   private static final String SVO_FILE_NAME = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20240715_103234_ZEDRecording_NewONRCourseWalk.svo2")
                                                                                        .toAbsolutePath()
                                                                                        .toString();

   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "srt_video_streamer_demo");
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

   private final ROS2SRTSensorStreamer streamer = new ROS2SRTSensorStreamer();
   private final ZEDColorDepthImageRetriever imageRetriever;

   private volatile boolean running = true;
   private final Notification doneNotification = new Notification();

   private ROS2SRTVideoStreamingDemo()
   {
      imageRetriever = new ZEDColorDepthImageRetriever(0, ReferenceFrame::getWorldFrame, () -> true, () -> true);
//      imageRetriever = new ZEDColorDepthImageRetrieverSVO(0, () -> true, () -> true, ros2Helper, RecordMode.PLAYBACK, SVO_FILE_NAME);
      imageRetriever.start();

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, getClass().getSimpleName() + "Destruction"));

      while (running)
      {
         RawImage leftColorImage = imageRetriever.getLatestRawColorImage(RobotSide.LEFT);
         RawImage rightColorImage = imageRetriever.getLatestRawColorImage(RobotSide.RIGHT);
         RawImage depthImage = imageRetriever.getLatestRawDepthImage();

         if (!streamer.hasStream(PerceptionAPI.ZED_DEPTH_STREAM))
         {
            streamer.addStream(PerceptionAPI.ZED_LEFT_COLOR_STREAM, leftColorImage, avutil.AV_PIX_FMT_BGR0, opencv_imgproc.COLOR_BGR2BGRA, true);
            streamer.addStream(PerceptionAPI.ZED_RIGHT_COLOR_STREAM, rightColorImage, avutil.AV_PIX_FMT_BGR0, opencv_imgproc.COLOR_BGR2BGRA, true);
            streamer.addStream(PerceptionAPI.ZED_DEPTH_STREAM, depthImage, avutil.AV_PIX_FMT_GRAY16);
         }

         streamer.sendFrame(PerceptionAPI.ZED_LEFT_COLOR_STREAM, leftColorImage);
         streamer.sendFrame(PerceptionAPI.ZED_RIGHT_COLOR_STREAM, rightColorImage);
         streamer.sendFrame(PerceptionAPI.ZED_DEPTH_STREAM, depthImage);

         leftColorImage.release();
         rightColorImage.release();
         depthImage.release();
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
      new ROS2SRTVideoStreamingDemo();
   }
}
