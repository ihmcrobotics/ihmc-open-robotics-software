package us.ihmc.rdx.ui.graphics;

import gnu.trove.list.array.TLongArrayList;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import us.ihmc.codecs.demuxer.MP4VideoDemuxer;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.log.LogTools;
import us.ihmc.perception.opencv.OpenCVTools;

import java.awt.image.BufferedImage;
import java.io.*;
import java.util.Arrays;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

public class LogVideoLoader
{
   private final MP4VideoDemuxer demuxer;
   private final YUVPictureConverter converter = new YUVPictureConverter();
   private String filename;

   private String timestampFilename;
   private Mat image;
   private VideoCapture capture;
   private final ScheduledExecutorService executorService = Executors.newScheduledThreadPool(1);

   private long currentlyShowingRobottimestamp = 0;
   private long upcomingRobotTimestamp = 0;
   private int currentlyShowingIndex = 0;

   private File videoFile;

   private boolean hasTimebase;
   private long bmdTimeBaseNumerator;
   private long bmdTimeBaseDenominator;
   private boolean interlaced;
   private long[] robotTimestamps;
   private long[] videoTimestamps;

   public LogVideoLoader(String file, String timestampFile) throws IOException
   {
      this.filename = file;
      this.timestampFilename = timestampFile;

      videoFile = new File(file);
      if (!videoFile.exists())
      {
         throw new IOException("Cannot find video: " + videoFile);
      }

      parseTimestampData(new File(timestampFile));
      demuxer = new MP4VideoDemuxer(videoFile);
      LogTools.info("Demuxer: {}", demuxer);
   }

   public Mat loadNextFrameAsOpenCVMat(long timestamp)
   {
      if (timestamp >= currentlyShowingRobottimestamp && timestamp < upcomingRobotTimestamp)
      {
         return null;
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
         upcomingRobotTimestamp = robotTimestamps[currentlyShowingIndex + 1];
      }
      else
      {
         upcomingRobotTimestamp = currentlyShowingRobottimestamp;
      }

      if (previousTimestamp == videoTimestamp)
      {
         return null;
      }

      videoTimestamp = getVideoTimestamp(timestamp);
      //      }
      try
      {
         demuxer.seekToPTS(videoTimestamp);
         YUVPicture nextFrame = demuxer.getNextFrame();
         BufferedImage bufImage = converter.toBufferedImage(nextFrame);

         Mat mat = OpenCVTools.convertBufferedImageToMat(bufImage);

         return mat;
      }
      catch (IOException e)
      {
         LogTools.info("Frame Loading Exception.");
         e.printStackTrace();
      }
      return null;
   }

   private void parseTimestampData(File timestampFile) throws IOException
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
               bmdTimeBaseNumerator = Long.valueOf(line);
            }
            else
            {
               throw new IOException("Cannot read numerator");
            }

            if ((line = reader.readLine()) != null)
            {
               bmdTimeBaseDenominator = Long.valueOf(line);
            }
            else
            {
               throw new IOException("Cannot read denumerator");
            }
         }

         TLongArrayList robotTimestamps = new TLongArrayList();
         TLongArrayList videoTimestamps = new TLongArrayList();
         line = reader.readLine();
         line = reader.readLine();
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
      finally
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
         videoTimestamp = (videoTimestamp * bmdTimeBaseNumerator * demuxer.getTimescale()) / (bmdTimeBaseDenominator);
      }

      return videoTimestamp;
   }
}
