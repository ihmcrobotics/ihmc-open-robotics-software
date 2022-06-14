package us.ihmc.gdx.ui.graphics;

import gnu.trove.list.array.TLongArrayList;
import imgui.ImGui;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import us.ihmc.codecs.demuxer.MP4VideoDemuxer;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.gdx.ui.graphics.live.GDXOpenCVVideoVisualizer;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoOpenCVTools;

import java.awt.image.BufferedImage;
import java.io.*;
import java.util.Arrays;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKeyEx;

public class GDXLogVideoVisualizer extends GDXOpenCVVideoVisualizer
{
   private final MP4VideoDemuxer demuxer;

   private final YUVPictureConverter converter = new YUVPictureConverter();
   private String filename;

   private String timestampFilename;
   private Mat image;
   private VideoCapture cap;
   private final ScheduledExecutorService executorService = Executors.newScheduledThreadPool(1);

   private long currentlyShowingRobottimestamp = 0;

   private int currentlyShowingIndex = 0;
   private File videoFile;
   private boolean hasTimebase;
   private long bmdTimeBaseNum;
   private long bmdTimeBaseDen;
   private boolean interlaced;
   private long[] robotTimestamps;
   private long[] videoTimestamps;

   private int index = 0;

   public GDXLogVideoVisualizer(String file, String timestampFile) throws IOException
   {
      super(file, file, false);
      this.filename = file;
      this.timestampFilename = timestampFile;

      executorService.scheduleAtFixedRate(this::loadNextFrame, 10L, 10L, TimeUnit.MILLISECONDS);

      videoFile = new File(file);
      if (!videoFile.exists())
      {
         throw new IOException("Cannot find video: " + videoFile);
      }

      parseTimestampData(new File(timestampFile));

      demuxer = new MP4VideoDemuxer(videoFile);

      LogTools.info("Demuxer: {}", demuxer);
   }

   public void loadNextFrame()
   {

      if(demuxer == null) return;

      long timestamp = robotTimestamps[index];

      LogTools.info("Timestamp: {}", timestamp, index);

      if(index < robotTimestamps.length) index++;
//
      long videoTimestamp;
//      if (robotTimestamps.length > currentlyShowingIndex + 1 && robotTimestamps[currentlyShowingIndex + 1] == timestamp)
//      {
//         currentlyShowingIndex++;
//         videoTimestamp = videoTimestamps[currentlyShowingIndex];
//         currentlyShowingRobottimestamp = robotTimestamps[currentlyShowingIndex];
//
//      }
//      else
//      {
         videoTimestamp = getVideoTimestamp(timestamp);
         LogTools.info("Video Timestamps: {}", videoTimestamp);
//      }
      try
      {
         demuxer.seekToPTS(videoTimestamp);
         YUVPicture nextFrame = demuxer.getNextFrame();
         BufferedImage bufImage = converter.toBufferedImage(nextFrame);

         Mat mat = BytedecoOpenCVTools.convertBufferedImageToMat(bufImage);

         imshow("OpenCV Mat",mat);
         int code = waitKeyEx(10);
         if(code == 113) System.exit(0);

         LogTools.info("Frame Loaded: {} {} {}", bufImage.getHeight(), bufImage.getWidth(), videoTimestamp);
      }
      catch (IOException e)
      {
         LogTools.info("Frame Loading Exception.");
         e.printStackTrace();
      }
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
               bmdTimeBaseNum = Long.valueOf(line);
            }
            else
            {
               throw new IOException("Cannot read numerator");
            }

            if ((line = reader.readLine()) != null)
            {
               bmdTimeBaseDen = Long.valueOf(line);
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

   @Override
   public void renderImGuiWidgets()
   {
      executorService.shutdown();
      super.renderImGuiWidgets();
      ImGui.text(this.filename);
//      if (getHasReceivedOne())
//         getFrequencyPlot().renderImGuiWidgets();
   }
}
