package us.ihmc.perception.streaming;

import org.bytedeco.opencv.global.opencv_highgui;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.Notification;
import us.ihmc.log.LogTools;

import java.net.InetSocketAddress;

import static org.bytedeco.ffmpeg.global.avutil.AV_PIX_FMT_BGR24;

public class SRTVideoReceiverDemo
{
   private static final InetSocketAddress STREAMER_ADDRESS = InetSocketAddress.createUnresolved("127.0.0.1", 60001);

   private final SRTVideoReceiver videoSubscriber;
   private boolean shutdown = false;
   private final Notification shutdownReady = new Notification();

   private SRTVideoReceiverDemo()
   {
      videoSubscriber = new SRTVideoReceiver(AV_PIX_FMT_BGR24);
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "SRTSubscriberDemoDestruction"));
      run();
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
      shutdown = true;

      shutdownReady.blockingPoll();

      videoSubscriber.destroy();
   }

   public static void main(String[] args)
   {
      new SRTVideoReceiverDemo();
   }
}
