package us.ihmc.robotDataCommunication.gui;

import java.awt.Dimension;
import java.awt.Rectangle;
import java.awt.Toolkit;
import java.io.IOException;
import java.net.InetAddress;
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
import us.ihmc.multicastLogDataProtocol.LogDataProtocolSettings;
import us.ihmc.multicastLogDataProtocol.SingleThreadMultiClientStreamingDataTCPServer;
import us.ihmc.robotDataCommunication.LogDataHeader;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.math.TimeTools;

public class GUICaptureStreamer
{
   private final JFrame window;
   private final int fps;

   private final ScreenCapture screenCapture = ScreenCaptureFactory.getScreenCapture();

   private final ScheduledThreadPoolExecutor scheduler = new ScheduledThreadPoolExecutor(1, ThreadTools.getNamedThreadFactory("GUICaptureStreamer"));
   private final CaptureRunner captureRunner = new CaptureRunner();
   private final Dimension size = new Dimension();

   private final SingleThreadMultiClientStreamingDataTCPServer server;
   private final GUICaptureBroadcast broadcast;
   private JPEGEncoder encoder = new JPEGEncoder();

   public GUICaptureStreamer(JFrame window, int fps, float quality, String hostToBindTo, InetAddress group)
   {

      this.window = window;
      this.fps = fps;
      try
      {
         server = new SingleThreadMultiClientStreamingDataTCPServer(LogDataProtocolSettings.UI_DATA_PORT);
         broadcast = new GUICaptureBroadcast(InetAddress.getByName(hostToBindTo), group.getAddress());
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

   }

   public synchronized void start()
   {
      scheduler.remove(captureRunner);
      scheduler.scheduleAtFixedRate(captureRunner, 10, TimeTools.nano / fps, TimeUnit.NANOSECONDS);
      broadcast.start();
      server.start();
   }

   public synchronized void stop()
   {
      scheduler.remove(captureRunner);
      broadcast.stop();
      server.close();
   }

   private class CaptureRunner implements Runnable
   {
      private ByteBuffer directBuffer;

      private ByteBuffer getOrCreateBuffer(int size)
      {
         if (directBuffer == null)
         {
            directBuffer = ByteBuffer.allocateDirect(size);
         }
         else if (directBuffer.capacity() < size)
         {
            directBuffer = ByteBuffer.allocateDirect(size);
         }

         directBuffer.clear();
         return directBuffer;
      }

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
               int dataLength = buffer.remaining();
               ByteBuffer sendBuffer = getOrCreateBuffer(dataLength + LogDataHeader.length());
               LogDataHeader header = new LogDataHeader();
               header.setUid(0);
               header.setDataSize(dataLength);
               header.setTimestamp(System.nanoTime());
               header.setCrc32(0);
               header.writeBuffer(0, sendBuffer);
               sendBuffer.position(LogDataHeader.length());
               sendBuffer.put(buffer);
               sendBuffer.flip();
               server.send(sendBuffer);
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
