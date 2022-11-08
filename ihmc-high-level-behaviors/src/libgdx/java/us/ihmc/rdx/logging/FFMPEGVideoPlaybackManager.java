package us.ihmc.rdx.logging;

import org.bytedeco.ffmpeg.avutil.AVRational;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.tools.thread.Throttler;

public class FFMPEGVideoPlaybackManager
{
   private final FFMPEGFileReader file;
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
      this.file = new FFMPEGFileReader(file);
      timeBase = this.file.getTimeBase();

      image = new BytedecoImage(getWidth(), getHeight(), opencv_core.CV_8UC4, this.file.getFrameDataBuffer());

      treatAsStream = this.file.getDuration() < 0;
   }

   public void seek(long milliseconds)
   {
      long returnCode = file.seek(millisToBaseUnits(milliseconds) + file.getStartTime());

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
      double period = 1 / FFMPEGTools.rationalToFloatingPoint(file.getFramerate());

      while (!isPaused)
      {
         long returnCode = file.getNextFrame();

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

   public boolean hasDuration() {
      return !treatAsStream;
   }

   public long getVideoDurationInMillis()
   {
      return baseUnitsToMillis(file.getDuration());
   }

   public long getCurrentTimestampInMillis()
   {
      return baseUnitsToMillis(previousBaseUnitsTimestamp - file.getStartTime());
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
