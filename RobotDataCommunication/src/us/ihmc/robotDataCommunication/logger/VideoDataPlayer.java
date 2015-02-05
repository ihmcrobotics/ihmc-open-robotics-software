package us.ihmc.robotDataCommunication.logger;

import gnu.trove.list.array.TLongArrayList;

import java.awt.Dimension;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.util.Arrays;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.SwingUtilities;

import us.ihmc.codecs.demuxer.MP4VideoDemuxer;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.robotDataCommunication.logger.util.ExternalProgramHelpers;
import us.ihmc.robotDataCommunication.logger.util.FFMpeg;
import us.ihmc.robotDataCommunication.logger.util.PipedCommandExecutor;
import us.ihmc.utilities.operatingSystem.OperatingSystem;

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
   private final HideableMediaFrame viewer;

   private final YUVPictureConverter converter = new YUVPictureConverter();

   private int currentlyShowingIndex = 0;
   private long currentlyShowingRobottimestamp = 0;
   private long upcomingRobottimestamp = 0;

   private final File videoFile;

   public VideoDataPlayer(String name, File dataDirectory, LogProperties logProperties) throws IOException
   {
      this.name = name;
      this.interlaced = logProperties.getInterlaced(name);
      this.hasTimebase = logProperties.hasTimebase();

      if (!hasTimebase)
      {
         System.err.println("Video data is using timestamps instead of frame numbers. Falling back to seeking based on timestamp.");
      }
      videoFile = new File(dataDirectory, logProperties.getVideoFile(name));
      if (!videoFile.exists())
      {
         throw new IOException("Cannot find video: " + videoFile);
      }

      File timestampFile = new File(dataDirectory, logProperties.getTimestampFile(name));

      parseTimestampData(timestampFile);

      demuxer = new MP4VideoDemuxer(videoFile);

      viewer = new HideableMediaFrame(name, demuxer.getWidth(), demuxer.getHeight());
   }

   public synchronized void showVideoFrame(long timestamp)
   {
      if (timestamp >= currentlyShowingRobottimestamp && timestamp < upcomingRobottimestamp)
      {
         return;
      }

      long videoTimestamp = getVideoTimestamp(timestamp);

      if (currentlyShowingIndex + 1 < robotTimestamps.length)
      {
         upcomingRobottimestamp = robotTimestamps[currentlyShowingIndex + 1];
      }
      else
      {
         upcomingRobottimestamp = currentlyShowingRobottimestamp;
      }

      try
      {
         demuxer.seekToPTS(videoTimestamp);
         YUVPicture nextFrame = demuxer.getNextFrame();
         viewer.update(nextFrame);
         nextFrame.delete();
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

   private void parseTimestampData(File timestampFile)
   {
      BufferedReader reader = null;
      try
      {
         reader = new BufferedReader(new FileReader(timestampFile));

         String line;
         if (hasTimebase)
         {
            if ((line = reader.readLine()) != null)
            {
               bmdTimeBaseNum = Long.valueOf(line);
            }
            else
            {
               throw new RuntimeException("Cannot read numerator");
            }

            if ((line = reader.readLine()) != null)
            {
               bmdTimeBaseDen = Long.valueOf(line);
            }
            else
            {
               throw new RuntimeException("Cannot read denumerator");
            }
         }

         TLongArrayList robotTimestamps = new TLongArrayList();
         TLongArrayList videoTimestamps = new TLongArrayList();
         while ((line = reader.readLine()) != null)
         {
            String[] stamps = line.split("\\s");
            long robotStamp = Long.valueOf(stamps[0]);
            long videoStamp = Long.valueOf(stamps[1]);

            if (interlaced)
            {
               videoStamp /= 2;
            }

            robotTimestamps.add(robotStamp);
            videoTimestamps.add(videoStamp);

         }

         this.robotTimestamps = robotTimestamps.toArray();
         this.videoTimestamps = videoTimestamps.toArray();

      }
      catch (FileNotFoundException e)
      {
         throw new RuntimeException(e);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      } finally
      {
         try
         {
            if (reader != null)
            {
               reader.close();
            }
         }
         catch (IOException e)
         {
         }
      }
   }

   public void exportVideo(File selectedFile, long startTimestamp, long endTimestamp, PrintStream output)
   {

      long startVideoTimestamp = getVideoTimestamp(startTimestamp);
      long endVideoTimestamp = getVideoTimestamp(endTimestamp);

      double timebase = 1.0 / ((double) demuxer.getTimescale());

      double startTime = startVideoTimestamp * timebase;
      double endTime = endVideoTimestamp * timebase;

      FFMpeg ffMpeg = new FFMpeg();

      ffMpeg.setStarttime(startTime);
      ffMpeg.setEndtime(endTime);
      ffMpeg.setInputFile("\"" + videoFile.getAbsolutePath() + "\"");
      ffMpeg.setVideoCodec("h264");
      ffMpeg.setAudioCodec("aac");
      ffMpeg.enableExperimentalCodecs(true);
      ffMpeg.setOutputFile("\"" + selectedFile.getAbsolutePath() + "\"");

      PipedCommandExecutor executor = new PipedCommandExecutor(ffMpeg);
      try
      {
         executor.execute(output, output);
         executor.waitFor();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

   }

   public void cropVideo(File outputFile, File timestampFile, long startTimestamp, long endTimestamp, PrintStream output) throws IOException
   {

      PrintWriter timestampWriter = new PrintWriter(timestampFile);
      timestampWriter.println(bmdTimeBaseNum);
      timestampWriter.println(bmdTimeBaseDen);

      long videoOffset = Long.MIN_VALUE;

      for (int i = 0; i < robotTimestamps.length; i++)
      {
         long robotTimestamp = robotTimestamps[i];
         long videoTimestamp = videoTimestamps[i];

         if (robotTimestamp >= startTimestamp && robotTimestamp <= endTimestamp)
         {
            if (videoOffset == Long.MIN_VALUE)
            {
               videoOffset = videoTimestamp;
            }

            timestampWriter.print(robotTimestamp);
            timestampWriter.print(" ");
            timestampWriter.println(videoTimestamp - videoOffset);
         }
         else if (robotTimestamp > endTimestamp)
         {
            break;
         }
      }

      timestampWriter.close();

      long startVideoTimestamp = getVideoTimestamp(startTimestamp);
      long endVideoTimestamp = getVideoTimestamp(endTimestamp);
      double timebase = 1.0 / ((double) demuxer.getTimescale());

      double startTime = startVideoTimestamp * timebase;
      double endTime = endVideoTimestamp * timebase;

      FFMpeg ffMpeg = new FFMpeg();

      ffMpeg.setStarttime(startTime);
      ffMpeg.setEndtime(endTime);

      String filePathEncloser;

      if (ExternalProgramHelpers.getOS() == OperatingSystem.WINDOWS)
      {
         filePathEncloser = "\"";
      }
      else
      {
         filePathEncloser = "'";
      }

      ffMpeg.setInputFile(filePathEncloser + videoFile.getAbsolutePath() + filePathEncloser);
      ffMpeg.setVideoCodec("copy");
      ffMpeg.setAudioCodec("copy");
      ffMpeg.enableExperimentalCodecs(true);
      ffMpeg.setOutputFile(filePathEncloser + outputFile.getAbsolutePath() + filePathEncloser);

      PipedCommandExecutor executor = new PipedCommandExecutor(ffMpeg);
      try
      {
         executor.execute(output, output);
         executor.waitFor();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private class HideableMediaFrame extends JFrame
   {
      private static final long serialVersionUID = -3494797002318746347L;
      final JLabel label = new JLabel();
      private BufferedImage img;
      private int width, height;

      public HideableMediaFrame(String name, int width, int height)
      {
         super(name);
         label.setPreferredSize(new Dimension(width, height));
         getContentPane().add(label);
         this.width = width;
         this.height = height;
         pack();
      }

      public void update(final YUVPicture nextFrame)
      {
         SwingUtilities.invokeLater(new Runnable()
         {
            @Override
            public void run()
            {
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
}
