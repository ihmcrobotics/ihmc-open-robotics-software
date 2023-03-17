package us.ihmc.rdx.logging;

import org.bytedeco.ffmpeg.avutil.AVRational;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.logging.HDF5Tools;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.tools.thread.Throttler;

public class FFMPEGVideoPlaybackManager
{
   private final IFFMPEGFileReader file;
   private final BytedecoImage image;
   private final AVRational timeBase;
   private long previousBaseUnitsTimestamp;
   private boolean isPaused = true;
   private boolean treatAsStream;
   private final Throttler throttler = new Throttler();
   private final Runnable playbackThreadRunnable = this::playbackThread;
   private Thread currentPlaybackThread;

   public FFMPEGVideoPlaybackManager(String file)
   {
      if (file.endsWith(PerceptionLoggerConstants.HDF5_FILE_EXTENSION))
         this.file = new FFMPEGHDF5FileReader(file);
      else
         this.file = new FFMPEGFileReader(file);

      timeBase = this.file.getTimeBase();

      image = new BytedecoImage(getWidth(), getHeight(), opencv_core.CV_8UC4, this.file.getFrameDataBuffer());

      treatAsStream = this.file.getDuration() < 0;
   }

   public void seekFrame(long frameNumber)
   {
      seek((long) Math.floor(frameNumber * Conversions.secondsToMilliseconds(calculateVideoFramePeriod())));
   }

   public void seek(long timebaseUnit)
   {
//      pause(); // Currently pausing is required to seek

      long returnCode = file.seek(timebaseUnit + file.getStartTime());

      if (returnCode == -1) //EOF
      {
         isPaused = true;
         return;
      }

      previousBaseUnitsTimestamp = returnCode;
   }

   public void play()
   {
      if (!isPaused)
         return;

      currentPlaybackThread = ThreadTools.startAThread(playbackThreadRunnable, "Video playback");

      isPaused = false;
   }

   private void playbackThread()
   {
      double period = calculateVideoFramePeriod();

      while (!isPaused)
      {
         long returnCode = file.getNextFrame(true);

         if (returnCode == -1) //EOF
         {
            isPaused = true;
            break;
         }

         previousBaseUnitsTimestamp = returnCode;

         throttler.waitAndRun(period);
      }
   }

   public void pause()
   {
      if (isPaused || currentPlaybackThread == null)
         return;

      isPaused = true;

      try
      {
         currentPlaybackThread.join();
      }
      catch (InterruptedException ex)
      {
         LogTools.error(ex);
      }
   }

   public double calculateTimeBaseSeconds()
   {
      return FFMPEGTools.rationalToFloatingPoint(file.getTimeBase());
   }

   public double calculateAverageFramerateHz()
   {
      return FFMPEGTools.rationalToFloatingPoint(file.getAverageFramerate());
   }

   public double calculateVideoFramePeriod()
   {
      return 1.0 / calculateAverageFramerateHz();
   }

   public IFFMPEGFileReader getFile()
   {
      return file;
   }

   public double calculateVideoDuration()
   {
      return file.getDuration() * calculateTimeBaseSeconds();
   }

   public double calculateNumberOfFrames()
   {
      return calculateVideoDuration() / calculateVideoFramePeriod();
   }

   public boolean isAStream()
   {
      return treatAsStream;
   }

   public long getVideoDurationInMillis()
   {
      return file.getDuration();
   }

   public long getCurrentTimestampInMillis()
   {
      return previousBaseUnitsTimestamp - file.getStartTime();
   }

   public double getCurrentTimestamp()
   {
      return Conversions.millisecondsToSeconds(getCurrentTimestampInMillis());
   }

   private long millisToBaseUnits(long millis)
   {
      return millis;
      //return (long) (millis / 1000 / FFMPEGTools.rationalToFloatingPoint(timeBase));
   }

   private long baseUnitsToMillis(long baseUnits)
   {
      return baseUnits;

      //      long millis = 0;
      //      if (timeBase.num() == 1) //Should basically always be one, but just to be safe
      //      {
      //         //This method increases accuracy when the time base is simple (which is often the case)
      //         millis = baseUnits / timeBase.den() * 1000;
      //         millis += (baseUnits % timeBase.den()) * FFMPEGTools.rationalToFloatingPoint(timeBase) * 1000;
      //      }
      //      else
      //      {
      //         millis = (long) (baseUnits * FFMPEGTools.rationalToFloatingPoint(timeBase) * 1000);
      //      }
      //
      //      return millis;
   }

   public int getWidth()
   {
      return file.getWidth();
   }

   public int getHeight()
   {
      return file.getHeight();
   }

   public BytedecoImage getImage()
   {
      return image;
   }

   public boolean isPaused()
   {
      return isPaused;
   }

   public void close()
   {
      pause();
      file.close();
   }
}
