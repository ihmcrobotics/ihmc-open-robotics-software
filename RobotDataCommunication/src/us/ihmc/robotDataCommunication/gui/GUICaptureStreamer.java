package us.ihmc.robotDataCommunication.gui;

import java.awt.Dimension;
import java.awt.Rectangle;
import java.awt.Toolkit;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.nio.ByteBuffer;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

import javax.imageio.IIOImage;
import javax.imageio.ImageIO;
import javax.imageio.ImageWriteParam;
import javax.imageio.ImageWriter;
import javax.swing.JFrame;

import us.ihmc.codecs.builder.ByteBufferImageOutputStream;
import us.ihmc.codecs.screenCapture.ScreenCapture;
import us.ihmc.codecs.screenCapture.ScreenCaptureFactory;
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

   private final ImageWriter writer = ImageIO.getImageWritersByFormatName("jpeg").next();
   private final ImageWriteParam param = writer.getDefaultWriteParam();

   private ByteBufferImageOutputStream imageOutputStream;

   private ByteBuffer buffer;
   private final Dimension size = new Dimension();

   private final SegmentedDatagramServer server;

   
   public GUICaptureStreamer(JFrame window, int fps, float quality, String hostToBindTo, InetAddress group, int port)
   {
      this(window, fps, quality, LogUtils.getMyInterface(hostToBindTo), group, port);
   }

   public GUICaptureStreamer(JFrame window, int fps, float quality, NetworkInterface iface, InetAddress group, int port)
   {
      this.window = window;
      this.fps = fps;
      param.setCompressionMode(ImageWriteParam.MODE_EXPLICIT);
      param.setCompressionQuality(quality);

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
            buffer = ByteBuffer.allocate(windowSize.width * windowSize.height * 4 * 4);
            imageOutputStream = new ByteBufferImageOutputStream(buffer);
            writer.setOutput(imageOutputStream);
            size.setSize(windowSize);
         }

         try
         {
            BufferedImage img = screenCapture.createScreenCapture(captureRectangle);
            if(img != null)
            {
               buffer.clear();
               writer.write(null, new IIOImage(img, null, null), param);
               buffer.flip();
   
               server.send(LogDataType.VIDEO, System.nanoTime(), buffer);
            }
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }

   }
}
