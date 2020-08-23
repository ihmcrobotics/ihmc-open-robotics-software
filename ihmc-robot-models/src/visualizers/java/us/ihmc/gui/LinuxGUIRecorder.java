package us.ihmc.gui;

import com.esotericsoftware.kryo.io.ByteBufferInputStream;
import us.ihmc.codecs.builder.MP4MJPEGMovieBuilder;
import us.ihmc.codecs.generated.RGBPicture;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.generated.YUVPicture.YUVSubsamplingType;
import us.ihmc.codecs.screenCapture.ScreenCapture;
import us.ihmc.codecs.screenCapture.ScreenCaptureFactory;
import us.ihmc.codecs.yuv.JPEGEncoder;
import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.thread.PausablePeriodicThread;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.file.FileVisitResult;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Comparator;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.function.Supplier;

// records rectangle of Linux X server and saves to file
public class LinuxGUIRecorder
{
   public static final int MAXIMUM_IMAGE_DATA_SIZE = 1024 * 1024;
   public static final String LOG_MP4_POSTFIX = "Log.mp4";
   private final Supplier<Rectangle> windowBoundsProvider;
   private final int fps;
   private final int quality;
   private final String guiName;
   private String filename;

   private final ScreenCapture screenCapture = ScreenCaptureFactory.getScreenCapture();

   private final PausablePeriodicThread scheduler;
   private final CaptureRunner captureRunner = new CaptureRunner();
   private final Dimension size = new Dimension();

   private JPEGEncoder encoder = new JPEGEncoder();
   private MP4MJPEGMovieBuilder builder;

   public LinuxGUIRecorder(Supplier<Rectangle> windowBoundsProvider, int fps, double quality, String guiName)
   {
      this.windowBoundsProvider = windowBoundsProvider;
      this.fps = fps;
      this.quality = (int) (100.0 * quality);
      this.guiName = guiName;

      scheduler = new PausablePeriodicThread("LinuxGUIRecorder", 1.0 / fps, captureRunner);
   }

   public void deleteOldLogs(int numberOflogsToKeep)
   {
      String defaultLogsDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;
      SortedSet<Path> sortedSet = new TreeSet<>(Comparator.comparing(path1 -> path1.getFileName().toString()));
      PathTools.walkFlat(Paths.get(defaultLogsDirectory), (path, type) -> {
         if (type == BasicPathVisitor.PathType.FILE && path.getFileName().toString().endsWith(guiName + LOG_MP4_POSTFIX))
            sortedSet.add(path);
         return FileVisitResult.CONTINUE;
      });

      while (sortedSet.size() > numberOflogsToKeep)
      {
         Path earliestLogDirectory = sortedSet.first();
         LogTools.warn("Deleting old log {}", earliestLogDirectory);
         FileTools.deleteQuietly(earliestLogDirectory);
         sortedSet.remove(earliestLogDirectory);
      }
   }

   public synchronized void start()
   {
      DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmssSSS");
      Calendar calendar = Calendar.getInstance();
      String timestamp = dateFormat.format(calendar.getTime());

      filename = System.getProperty("user.home") + "/.ihmc/logs/" + timestamp + "_" + guiName + LOG_MP4_POSTFIX;

      LogTools.info("Starting recording to {}", filename);

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

      scheduler.start();
   }

   public synchronized void stop()
   {
      if (filename != null)
         LogTools.info("Stopping recording to {}", filename);

      scheduler.stop();

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
            RGBPicture screenCapture = LinuxGUIRecorder.this.screenCapture.createScreenCapture(captureRectangle);

            if (screenCapture != null)
            {
               YUVPicture yuv = screenCapture.toYUV(YUVSubsamplingType.YUV420);
               ByteBuffer buffer = encoder.encode(yuv, quality);
               if (buffer.remaining() <= MAXIMUM_IMAGE_DATA_SIZE)
               {
                  // write buffer to file
                  if (builder == null)
                  {
                     try
                     {
                        // Decode the first image using ImageIO, to get the dimensions
                        ByteBufferInputStream inputStream = new ByteBufferInputStream(buffer);
                        BufferedImage readImg = ImageIO.read(inputStream);
                        if (readImg == null)
                        {
                           LogTools.error("Cannot decode image");
                           return;
                        }
                        File videoFileFile = new File(filename);
                        builder = new MP4MJPEGMovieBuilder(videoFileFile, readImg.getWidth(), readImg.getHeight(), fps, quality);
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
                     e.printStackTrace();
                  }
               }
               else
               {
                  System.err.println("Not sending screen capture, image size exceeds " + MAXIMUM_IMAGE_DATA_SIZE);
               }
               yuv.delete();
               screenCapture.delete();
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
      scheduler.stop();
   }
}
