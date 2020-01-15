package us.ihmc.gui;

import us.ihmc.codecs.builder.MP4MJPEGMovieBuilder;
import us.ihmc.codecs.generated.RGBPicture;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.generated.YUVPicture.YUVSubsamplingType;
import us.ihmc.codecs.screenCapture.ScreenCapture;
import us.ihmc.codecs.screenCapture.ScreenCaptureFactory;
import us.ihmc.codecs.yuv.JPEGEncoder;
import us.ihmc.commons.thread.ThreadTools;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

// records rectangle of Linux X server and saves to file
public class LinuxGUIRecorder
{
   public static final int MAXIMUM_IMAGE_DATA_SIZE = 1024 * 1024;
   private final Supplier<Rectangle> windowBoundsProvider;
   private final int fps;
   private String filename;

   private final ScreenCapture screenCapture = ScreenCaptureFactory.getScreenCapture();

   private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory("GUICaptureStreamer"));
   private final CaptureRunner captureRunner = new CaptureRunner();
   private final Dimension size = new Dimension();

   private JPEGEncoder encoder = new JPEGEncoder();

   private ScheduledFuture<?> future = null;

   private MP4MJPEGMovieBuilder builder;

   public LinuxGUIRecorder(Supplier<Rectangle> windowBoundsProvider, int fps, float quality, String filename)
   {
      this.windowBoundsProvider = windowBoundsProvider;
      this.fps = fps;
      this.filename = filename;
   }

   public synchronized void start()
   {
      DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
      Calendar calendar = Calendar.getInstance();
      String timestamp = dateFormat.format(calendar.getTime());

      filename = System.getProperty("user.home") + "/robotLogs/" + timestamp + "_UILog";

      try
      {
         if (builder != null)
         {
            builder.close();
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      builder = null;

      if (future != null)
      {
         future.cancel(false);
      }

      scheduler.scheduleAtFixedRate(captureRunner, 10, 1000000000 / fps, TimeUnit.NANOSECONDS);
   }

   public synchronized void stop()
   {
      if (future != null)
      {
         future.cancel(false);
      }

      try
      {
         if (builder != null)
         {
            builder.close();
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      builder = null;
   }

   private class CaptureRunner implements Runnable
   {
      @Override
      public void run()
      {
         Rectangle windowBounds = windowBoundsProvider.get();
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
               if (buffer.remaining() <= MAXIMUM_IMAGE_DATA_SIZE)
               {
                  // write buffer to file
                  if (builder == null)
                  {
                     try
                     {
                        // Decode the first image using ImageIO, to get the dimensions
                        ByteArrayInputStream inputStream = new ByteArrayInputStream(buffer.array());
                        BufferedImage readImg = ImageIO.read(inputStream);
                        if (readImg == null)
                        {
                           System.err.println("Cannot decode image");
                           return;
                        }
                        File videoFileFile = new File(filename);
                        builder = new MP4MJPEGMovieBuilder(videoFileFile, readImg.getWidth(), readImg.getHeight(), 10, 90);
                        buffer.clear();
                     }
                     catch (IOException e)
                     {
                        e.printStackTrace();
                        return;
                     }

                  }
                  try
                  {
                     builder.encodeFrame(buffer);
                  }
                  catch (IOException e)
                  {
                     // TODO Auto-generated catch block
                     e.printStackTrace();
                  }
               }
               else
               {
                  System.err.println("Not sending screen capture, image size exceeds " + MAXIMUM_IMAGE_DATA_SIZE);
               }
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

   public void destroy()
   {
      scheduler.shutdownNow();
   }
}
