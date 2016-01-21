package us.ihmc.robotDataCommunication.logger;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import us.ihmc.javadecklink.Capture;
import us.ihmc.javadecklink.CaptureHandler;
import us.ihmc.robotics.time.TimeTools;
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

   public BlackmagicVideoDataLogger(File logPath, LogProperties logProperties, VideoSettings settings, YoVariableLoggerOptions options) throws IOException
   {
      super(logPath, logProperties, settings.getDescription(), settings.isInterlaced());
      decklink = settings.getDevice();
      quality = 0.75;

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
    * @see us.ihmc.robotDataCommunication.logger.VideoDataLoggerInterface#restart()
    */
   @Override
   public void restart() throws IOException
   {
      close();
      removeLogFiles();
      createCaptureInterface();
   }

   /* (non-Javadoc)
    * @see us.ihmc.robotDataCommunication.logger.VideoDataLoggerInterface#timestampChanged(long)
    */
   @Override
   public void timestampChanged(long newTimestamp)
   {
      if(capture != null)
      {
         long hardwareTimestamp = capture.getHardwareTime();
         circularLongMap.insert(hardwareTimestamp, newTimestamp);
      }
   }

   /* (non-Javadoc)
    * @see us.ihmc.robotDataCommunication.logger.VideoDataLoggerInterface#close()
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
   public void receivedFrameAtTime(long hardwareTime, long pts)
   {
      if(frame % 60 == 0)
      {
         System.out.println("[Decklink " + decklink + "] Received frame " + frame + " at time " + hardwareTime + "ns, delay: " + TimeTools.nanoSecondstoSeconds(circularLongMap.getLatestValue() - hardwareTime) + "s. pts: " + pts);
      }
      
      if(circularLongMap.size() > 0)
      {  
         long robotTimestamp = circularLongMap.getValue(true, hardwareTime);
         
         try
         {
            timestampWriter.write(robotTimestamp + " " + pts);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
      
      ++frame;
   }



}
