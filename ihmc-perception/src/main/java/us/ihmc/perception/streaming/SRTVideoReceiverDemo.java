package us.ihmc.perception.streaming;

import org.bytedeco.opencv.global.opencv_highgui;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.Notification;
import us.ihmc.log.LogTools;

import java.net.InetSocketAddress;

import static org.bytedeco.ffmpeg.global.avutil.AV_PIX_FMT_BGR24;

/**
 * <p>
 * Demo for the {@link SRTVideoReceiver}. Connects to a streamer
 * on the {@link #STREAMER_ADDRESS}, and shows the video using
 * OpenCV HighGUI
 * </p>
 * <p>
 * To stream a video for this demo, use the following command:
 * {@code ffmpeg -re -i [video-file] -c:v h264_nvenc -f mpegts -mode listener srt://127.0.0.1:60001}.
 * Alternatively, you can run the {@link SRTVideoStreamerDemo},
 * but be sure the set the {@link #STREAMER_ADDRESS} to the one printed by the streamer demo.
 * </p>
 */
public class SRTVideoReceiverDemo
{
   /* package-private */ static final InetSocketAddress STREAMER_ADDRESS = InetSocketAddress.createUnresolved("127.0.0.1", 60001);

   private final SRTVideoReceiver videoSubscriber;
   private volatile boolean shutdown = false;
   private final Notification shutdownReady = new Notification();

   private SRTVideoReceiverDemo()
   {
      videoSubscriber = new SRTVideoReceiver(AV_PIX_FMT_BGR24);
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "SRTSubscriberDemoDestruction"));
   }

   private void run()
   {
      if (!videoSubscriber.connect(STREAMER_ADDRESS, 10.0))
      {
         LogTools.info("Could not establish a connection with {}", STREAMER_ADDRESS.getHostString());
         shutdownReady.set();
         return;
      }

      LogTools.info("Connected to {}", STREAMER_ADDRESS.getHostString());

      while (!shutdown && videoSubscriber.isConnected())
      {
         Mat frame = videoSubscriber.getNextFrame(0.5);
         if (frame == null)
            break;

         opencv_highgui.imshow("Decoded Video", frame);
         opencv_highgui.waitKey(1);
         frame.close();
      }

      LogTools.info("Disconnecting from {}", STREAMER_ADDRESS.getHostString());
      videoSubscriber.disconnect();
      shutdownReady.set();
   }

   private void destroy()
   {
      if (shutdown)
         return;

      shutdown = true;

      shutdownReady.blockingPoll();

      videoSubscriber.destroy();
   }

   public static void main(String[] args)
   {
      SRTVideoReceiverDemo demo = new SRTVideoReceiverDemo();
      demo.run();
      demo.destroy();
   }
}
