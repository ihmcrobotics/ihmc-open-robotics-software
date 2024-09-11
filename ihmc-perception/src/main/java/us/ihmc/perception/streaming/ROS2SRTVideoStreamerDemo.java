package us.ihmc.perception.streaming;

import org.bytedeco.ffmpeg.global.avutil;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.perception.realsense.RealsenseDeviceManager;
import us.ihmc.sensors.RealsenseColorDepthImageRetriever;

public class ROS2SRTVideoStreamerDemo
{
   private final ROS2SRTVideoStreamer streamer = new ROS2SRTVideoStreamer(PerceptionAPI.REALSENSE_COLOR_STREAM);
   private final RealsenseDeviceManager realsenseManager = new RealsenseDeviceManager();
   private final RealsenseColorDepthImageRetriever imageRetriever = new RealsenseColorDepthImageRetriever(realsenseManager,
                                                                                                          "215122253249",
                                                                                                          RealsenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ,
                                                                                                          ReferenceFrame::getWorldFrame,
                                                                                                          () -> true);
   private volatile boolean running = true;
   private final Notification doneNotification = new Notification();
   private ROS2SRTVideoStreamerDemo()
   {
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, getClass().getSimpleName() + "Destruction"));

      while (running)
      {
         RawImage colorImage = imageRetriever.getLatestRawColorImage();
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
   }

   public static void main(String[] args)
   {
      new ROS2SRTVideoStreamerDemo();
   }
}
