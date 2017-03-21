package us.ihmc.robotDataLogger.logger;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import us.ihmc.commons.Conversions;
import us.ihmc.javadecklink.Capture;
import us.ihmc.javadecklink.CaptureHandler;
import us.ihmc.robotDataLogger.LogProperties;
import us.ihmc.tools.maps.CircularLongMap;

public class BlackmagicVideoDataLogger extends VideoDataLoggerInterface implements CaptureHandler
{

   /**
    * Make sure to set a progressive mode, otherwise the timestamps will be all wrong!
    */

   private final int decklink;
   private final double quality;
   private Capture capture;
   
   private final CircularLongMap circularLongMap = new CircularLongMap(10000);

   private FileWriter timestampWriter;
   
   private int frame;

   public BlackmagicVideoDataLogger(File logPath, LogProperties logProperties, int decklinkID, YoVariableLoggerOptions options) throws IOException
   {
      super(logPath, logProperties, "Camera" + decklinkID);
      decklink = decklinkID;
      quality = options.getVideoQuality();

      createCaptureInterface();
   }
   
   private void createCaptureInterface()
   {
      File timestampFile = new File(timestampData);
      capture = new Capture(this);
      try
      {
         timestampWriter = new FileWriter(timestampFile);
         capture.startCapture(videoFile, decklink, quality);
      }
      catch (IOException e)
      {
         capture = null;
         if(timestampWriter != null)
         {
            try
            {
               timestampWriter.close();
               timestampFile.delete();
            }
            catch (IOException e1)
            {
            }
         }
         timestampWriter = null;
         System.out.println("Cannot start capture interface");
         e.printStackTrace();
      }
   }
   
   /* (non-Javadoc)
    * @see us.ihmc.robotDataLogger.logger.VideoDataLoggerInterface#restart()
    */
   @Override
   public void restart() throws IOException
   {
      close();
      removeLogFiles();
      createCaptureInterface();
   }

   /* (non-Javadoc)
    * @see us.ihmc.robotDataLogger.logger.VideoDataLoggerInterface#timestampChanged(long)
    */
   @Override
   public void timestampChanged(long newTimestamp)
   {
      if(capture != null)
      {
         long hardwareTimestamp = capture.getHardwareTime();
         if(hardwareTimestamp != -1)
         {
            circularLongMap.insert(hardwareTimestamp, newTimestamp);
         }
      }
   }

   /* (non-Javadoc)
    * @see us.ihmc.robotDataLogger.logger.VideoDataLoggerInterface#close()
    */
   @Override
   public void close()
   {
      System.out.println("Signalling recorder");
      if(capture != null)
      {
         try
         {
            capture.stopCapture();
            timestampWriter.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         capture = null;
         timestampWriter = null;
      }
      
   }

   @Override
   public void receivedFrameAtTime(long hardwareTime, long pts, long timeScaleNumerator, long timeScaleDenumerator)
   {
      
      if(circularLongMap.size() > 0)
      {
         if(frame % 60 == 0)
         {
            System.out.println("[Decklink " + decklink + "] Received frame " + frame + " at time " + hardwareTime + "ns, delay: " + Conversions.nanosecondsToSeconds(circularLongMap.getLatestKey() - hardwareTime) + "s. pts: " + pts);
         }

         
         long robotTimestamp = circularLongMap.getValue(true, hardwareTime);
         
         try
         {
            if(frame == 0)
            {
               timestampWriter.write(timeScaleNumerator + "\n");
               timestampWriter.write(timeScaleDenumerator + "\n");
            }
            timestampWriter.write(robotTimestamp + " " + pts + "\n");
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         ++frame;
      }
      
   }



}
