package us.ihmc.robotDataCommunication.gui;

import java.awt.Dimension;
import java.awt.Rectangle;
import java.awt.Toolkit;
import java.io.IOException;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.nio.ByteBuffer;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

import javax.swing.JFrame;

import us.ihmc.codecs.generated.RGBPicture;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.generated.YUVPicture.YUVSubsamplingType;
import us.ihmc.codecs.screenCapture.ScreenCapture;
import us.ihmc.codecs.screenCapture.ScreenCaptureFactory;
import us.ihmc.codecs.yuv.JPEGEncoder;
import us.ihmc.multicastLogDataProtocol.LogDataType;
import us.ihmc.multicastLogDataProtocol.LogUtils;
import us.ihmc.multicastLogDataProtocol.SegmentedDatagramServer;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.math.TimeTools;

public class GUICaptureStreamer
{
   public static final int MAGIC_SESSION_ID = 0x81359;
   public static final int PORT = 12451;
   public static final InetAddress group = LogUtils.getByName("239.255.25.1");

   private final JFrame window;
   private final int fps;

   private final ScreenCapture screenCapture = ScreenCaptureFactory.getScreenCapture();

   private final ScheduledThreadPoolExecutor scheduler = new ScheduledThreadPoolExecutor(1, ThreadTools.getNamedThreadFactory("GUICaptureStreamer"));
   private final CaptureRunner captureRunner = new CaptureRunner();
   private final Dimension size = new Dimension();

   private final SegmentedDatagramServer server;
   private JPEGEncoder encoder = new JPEGEncoder();

   public GUICaptureStreamer(JFrame window, int fps, float quality, String hostToBindTo, InetAddress group, int port)
   {
      this(window, fps, quality, LogUtils.getMyInterface(hostToBindTo), group, port);
   }

   public GUICaptureStreamer(JFrame window, int fps, float quality, NetworkInterface iface, InetAddress group, int port)
   {
      this.window = window;
      this.fps = fps;
      try
      {
         server = new SegmentedDatagramServer(MAGIC_SESSION_ID, iface, group, port);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

   }

   public synchronized void start()
   {
      stop();
      scheduler.scheduleAtFixedRate(captureRunner, 10, TimeTools.nano / fps, TimeUnit.NANOSECONDS);
   }

   public synchronized void stop()
   {
      scheduler.remove(captureRunner);
   }

   private class CaptureRunner implements Runnable
   {

      @Override
      public void run()
      {
         Rectangle windowBounds = window.getBounds();
         Rectangle screen = new Rectangle(Toolkit.getDefaultToolkit().getScreenSize());
         Rectangle captureRectangle = windowBounds.intersection(screen);
         Dimension windowSize = captureRectangle.getSize();
         if (!windowSize.equals(size))
         {
            size.setSize(windowSize);
         }

         try
         {
            RGBPicture img = screenCapture.createScreenCapture(captureRectangle);

            if (img != null)
            {
               YUVPicture yuv = img.toYUV(YUVSubsamplingType.YUV420);
               ByteBuffer buffer = encoder.encode(yuv, 90);

               server.send(LogDataType.VIDEO, System.nanoTime(), buffer);
               yuv.delete();
               img.delete();
            }
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }

   }
}
