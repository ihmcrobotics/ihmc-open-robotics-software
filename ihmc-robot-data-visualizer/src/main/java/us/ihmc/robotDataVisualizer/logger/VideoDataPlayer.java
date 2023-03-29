package us.ihmc.robotDataVisualizer.logger;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Arrays;

import javax.swing.*;

import gnu.trove.list.array.TLongArrayList;
import javafx.scene.image.WritableImage;
import us.ihmc.codecs.demuxer.MP4VideoDemuxer;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.robotDataLogger.Camera;
import us.ihmc.robotDataVisualizer.logger.converters.VideoConverter;
import us.ihmc.robotDataVisualizer.logger.util.ProgressMonitorInterface;

import static java.lang.Long.parseLong;

public class VideoDataPlayer
{
   private final String name;
   private final boolean hasTimebase;
   private final boolean interlaced;

   private long[] robotTimestamps;
   private long[] videoTimestamps;

   private long bmdTimeBaseNum;
   private long bmdTimeBaseDen;

   private final MP4VideoDemuxer demuxer;
   private static HideableMediaFrame viewer;

   private final YUVPictureConverter converter = new YUVPictureConverter();

   private int currentlyShowingIndex = 0;
   private long currentlyShowingRobottimestamp = 0;
   private long upcomingRobottimestamp = 0;

   private final File videoFile;
   private final ConcurrentCopier<FrameData> imageBuffer = new ConcurrentCopier<>(FrameData::new);

   public VideoDataPlayer(Camera camera, File dataDirectory, boolean hasTimeBase) throws IOException
   {
      this.name = camera.getNameAsString();
      this.interlaced = camera.getInterlaced();
      this.hasTimebase = hasTimeBase;

      if (!hasTimebase)
      {
         System.err.println("Video data is using timestamps instead of frame numbers. Falling back to seeking based on timestamp.");
      }
      videoFile = new File(dataDirectory, camera.getVideoFileAsString());
      if (!videoFile.exists())
      {
         throw new IOException("Cannot find video: " + videoFile);
      }

      File timestampFile = new File(dataDirectory, camera.getTimestampFileAsString());

      parseTimestampData(timestampFile);

      demuxer = new MP4VideoDemuxer(videoFile);

      viewer = new HideableMediaFrame(camera.getNameAsString(), demuxer.getWidth(), demuxer.getHeight());
   }

   public synchronized void showVideoFrame(long timestamp)
   {
      if (timestamp >= currentlyShowingRobottimestamp && timestamp < upcomingRobottimestamp)
      {
         return;
      }

      long previousTimestamp = videoTimestamps[currentlyShowingIndex];

      long videoTimestamp;
      if (robotTimestamps.length > currentlyShowingIndex + 1 && robotTimestamps[currentlyShowingIndex + 1] == timestamp)
      {
         currentlyShowingIndex++;
         videoTimestamp = videoTimestamps[currentlyShowingIndex];
         currentlyShowingRobottimestamp = robotTimestamps[currentlyShowingIndex];

      }
      else
      {
         videoTimestamp = getVideoTimestamp(timestamp);

      }

      if (currentlyShowingIndex + 1 < robotTimestamps.length)
      {
         upcomingRobottimestamp = robotTimestamps[currentlyShowingIndex + 1];
      }
      else
      {
         upcomingRobottimestamp = currentlyShowingRobottimestamp;
      }

      if (previousTimestamp == videoTimestamp)
      {
         return;
      }

      try
      {
         demuxer.seekToPTS(videoTimestamp);
         YUVPicture nextFrame = demuxer.getNextFrame();

         FrameData copyForWriting = imageBuffer.getCopyForWriting();
         copyForWriting.cameraTargetPTS = videoTimestamp;
         copyForWriting.cameraCurrentPTS = demuxer.getCurrentPTS();
         copyForWriting.robotTimestamp = currentlyShowingRobottimestamp;

         viewer.update(nextFrame, copyForWriting);

         imageBuffer.commit();
      }
      catch (IOException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }

   }

   public void setVisible(boolean visible)
   {
      viewer.setVisible(visible);
   }

   private long getVideoTimestamp(long timestamp)
   {
      currentlyShowingIndex = Arrays.binarySearch(robotTimestamps, timestamp);

      if (currentlyShowingIndex < 0)
      {
         int nextIndex = -currentlyShowingIndex + 1;
         if ((nextIndex < robotTimestamps.length) && (Math.abs(robotTimestamps[-currentlyShowingIndex] - timestamp) > Math.abs(robotTimestamps[nextIndex])))
         {
            currentlyShowingIndex = nextIndex;
         }
         else
         {
            currentlyShowingIndex = -currentlyShowingIndex;
         }
      }

      if (currentlyShowingIndex < 0)
         currentlyShowingIndex = 0;
      if (currentlyShowingIndex >= robotTimestamps.length)
         currentlyShowingIndex = robotTimestamps.length - 1;
      currentlyShowingRobottimestamp = robotTimestamps[currentlyShowingIndex];

      long videoTimestamp = videoTimestamps[currentlyShowingIndex];

      if (hasTimebase)
      {
         videoTimestamp = (videoTimestamp * bmdTimeBaseNum * demuxer.getTimescale()) / (bmdTimeBaseDen);
      }

      return videoTimestamp;
   }

   private void parseTimestampData(File timestampFile) throws IOException
   {
      try (BufferedReader reader = new BufferedReader(new FileReader(timestampFile)))
      {
         String line;
         if (hasTimebase)
         {
            if ((line = reader.readLine()) != null)
            {
               bmdTimeBaseNum = parseLong(line);
            } else
            {
               throw new IOException("Cannot read numerator");
            }

            if ((line = reader.readLine()) != null)
            {
               bmdTimeBaseDen = parseLong(line);
            } else
            {
               throw new IOException("Cannot read denumerator");
            }
         }

         TLongArrayList robotTimestamps = new TLongArrayList();
         TLongArrayList videoTimestamps = new TLongArrayList();
         while ((line = reader.readLine()) != null)
         {
            String[] stamps = line.split("\\s");
            long robotStamp = parseLong(stamps[0]);
            long videoStamp = parseLong(stamps[1]);

            if (interlaced)
            {
               videoStamp /= 2;
            }

            robotTimestamps.add(robotStamp);
            videoTimestamps.add(videoStamp);

         }

         this.robotTimestamps = robotTimestamps.toArray();
         this.videoTimestamps = videoTimestamps.toArray();

      } catch (FileNotFoundException e)
      {
         throw new RuntimeException(e);
      }
   }

   public void exportVideo(File selectedFile, long startTimestamp, long endTimestamp, ProgressMonitorInterface monitor)
   {

      long startVideoTimestamp = getVideoTimestamp(startTimestamp);
      long endVideoTimestamp = getVideoTimestamp(endTimestamp);

      try
      {
         VideoConverter.convert(videoFile, selectedFile, startVideoTimestamp, endVideoTimestamp, monitor);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

   }

   public void cropVideo(File outputFile, File timestampFile, long startTimestamp, long endTimestamp, ProgressMonitorInterface monitor) throws IOException
   {

      long startVideoTimestamp = getVideoTimestamp(startTimestamp);
      long endVideoTimestamp = getVideoTimestamp(endTimestamp);

      int framerate = VideoConverter.crop(videoFile, outputFile, startVideoTimestamp, endVideoTimestamp, monitor);

      PrintWriter timestampWriter = new PrintWriter(timestampFile);
      timestampWriter.println(1);
      timestampWriter.println(framerate);

      long pts = 0;
      /*
       * PTS gets reorderd to be monotonically increaseing starting from 0
       */
      for (int i = 0; i < robotTimestamps.length; i++)
      {
         long robotTimestamp = robotTimestamps[i];

         if (robotTimestamp >= startTimestamp && robotTimestamp <= endTimestamp)
         {

            timestampWriter.print(robotTimestamp);
            timestampWriter.print(" ");
            timestampWriter.println(pts);
            pts++;
         }
         else if (robotTimestamp > endTimestamp)
         {
            break;
         }
      }

      timestampWriter.close();
   }

   private class HideableMediaFrame extends JFrame
   {
      private static final long serialVersionUID = -3494797002318746347L;
      final JLabel label = new JLabel();
      private BufferedImage img;
      private int width, height;
      JPanel panel = new JPanel(new BorderLayout());

      //actual data for the table in a 2d array
      Object[][] data = new Object[][]{
              {"cameraTargetPTS", 0},
              {"cameraCurrentPTS", 0},
              {"robotTimestamp", 0},
      };

      String[] columnNames = { "cameraTargetPTS", "cameraCurrentPTS', robotTimestamp"};

      JTable table = new JTable(data, columnNames);

      public HideableMediaFrame(String name, int width, int height)
      {
         super(name);
         label.setPreferredSize(new Dimension(width, height));
         getContentPane().add(label);
         this.width = width;
         this.height = height;

         panel.add(label, BorderLayout.CENTER);
         panel.setBackground(Color.WHITE);
         panel.add(table, BorderLayout.SOUTH);

         pack();
      }

      public JPanel getPanel()
      {
         return panel;
      }


      public void update(final YUVPicture nextFrame, FrameData timestampData)
      {
         SwingUtilities.invokeLater(new Runnable()
         {
            @Override
            public void run()
            {
               table.setValueAt(timestampData.cameraTargetPTS, 0, 1);
               table.setValueAt(timestampData.cameraCurrentPTS, 1, 1);
               table.setValueAt(timestampData.robotTimestamp, 2, 1);

               img = converter.toBufferedImage(nextFrame, img);
               nextFrame.delete();
               ImageIcon icon = new ImageIcon(img);
               label.setIcon(icon);

               if (img.getWidth() != width || img.getHeight() != height)
               {
                  width = img.getWidth();
                  height = img.getHeight();
                  label.setPreferredSize(new Dimension(width, height));
                  pack();
               }
            }
         });
      }
   }

   public String getName()
   {
      return name;
   }

   public static void main(String[] args) throws IOException
   {
      Camera camera = new Camera();
      camera.setName("test");
      camera.setInterlaced(false);
      String videoName = "ValkyrieTripodNorth";
      camera.setTimestampFile(videoName + "_Timestamps.dat");
      camera.setVideoFile(videoName + "_Video.mov");

      File dataDirectory = new File("//Gideon/LogData/LoggerDevLogs/Comparison/Valkyrie_GStreamer_HDMI_timestamps_getPTS/");

      VideoDataPlayer player = new VideoDataPlayer(camera, dataDirectory, true);

      for (int i = 1; i < player.robotTimestamps.length; i++)
      {
         if (player.robotTimestamps[i - 1] > player.robotTimestamps[i])
         {
            System.out.println("Non-monotonic robot timestamps");
            System.out.println(player.robotTimestamps[i - 1]);
         }
         if (player.videoTimestamps[i - 1] >= player.videoTimestamps[i])
         {
            System.out.println("Non-monotonic video timestamps");
            System.out.println(player.videoTimestamps[i - 1]);
         }

      }

      viewer.add(viewer.getPanel());

      viewer.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

      Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
      viewer.setSize(screenSize.width - 100, screenSize.height - 120);

      player.setVisible(true);

      long previousRobotTimestamp = player.robotTimestamps[0];
      long previousNanos = System.nanoTime();
      boolean playRealtime = true;
      
      for (int i = 1; i < player.robotTimestamps.length; i++)
      {
         long nextRobotTimestamp = player.robotTimestamps[i];
         long nextNanos = previousNanos + nextRobotTimestamp - previousRobotTimestamp;

         if (playRealtime)
         {
            while (System.nanoTime() < nextNanos);
         }
         else
         {
            ThreadTools.sleepSeconds(2);
         }

         previousNanos = System.nanoTime();

         System.out.println(player.videoTimestamps[i]);
         player.showVideoFrame(nextRobotTimestamp);
         previousRobotTimestamp = nextRobotTimestamp;
      }
   }


   public static class FrameData
   {
      public WritableImage frame;
      public long cameraTargetPTS;
      public long cameraCurrentPTS;
      public long robotTimestamp;
   }
}
