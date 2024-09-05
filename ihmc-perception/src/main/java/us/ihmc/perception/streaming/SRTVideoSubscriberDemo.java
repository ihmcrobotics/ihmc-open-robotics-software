package us.ihmc.perception.streaming;

import org.bytedeco.opencv.global.opencv_highgui;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.Notification;
import us.ihmc.log.LogTools;

import java.net.InetSocketAddress;

import static org.bytedeco.ffmpeg.global.avutil.AV_PIX_FMT_BGR24;

public class SRTVideoSubscriberDemo
{
   private static final InetSocketAddress CALLER_ADDRESS = InetSocketAddress.createUnresolved("127.0.0.1", 60001);

   private final SRTVideoSubscriber videoSubscriber;
   private boolean shutdown = false;
   private final Notification shutdownReady = new Notification();

   private SRTVideoSubscriberDemo()
   {
      videoSubscriber = new SRTVideoSubscriber(CALLER_ADDRESS, AV_PIX_FMT_BGR24);
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "SRTSubscriberDemoDestruction"));
      run();
   }

   private void run()
   {
      if (!videoSubscriber.waitForConnection(10.0))
      {
         LogTools.info("Could not establish a connection with {}", CALLER_ADDRESS.getHostString());
         shutdownReady.set();
         return;
      }

      LogTools.info("Connected to {}", CALLER_ADDRESS.getHostString());

      while (!shutdown && videoSubscriber.isConnected())
      {
         Mat frame = videoSubscriber.getNextImage(0.5);
         if (frame == null)
            break;

         opencv_highgui.imshow("Decoded Video", frame);
         opencv_highgui.waitKey(1);
         frame.close();
      }

      LogTools.info("Disconnecting from {}", CALLER_ADDRESS.getHostString());
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
      new SRTVideoSubscriberDemo();
   }
}
