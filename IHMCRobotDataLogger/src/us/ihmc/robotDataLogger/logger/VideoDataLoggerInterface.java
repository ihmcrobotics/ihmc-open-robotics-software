package us.ihmc.robotDataLogger.logger;

import java.io.File;
import java.io.IOException;

import us.ihmc.communication.net.TimestampListener;

public abstract class VideoDataLoggerInterface implements TimestampListener
{
   public static final String timestampDataPostfix = "Timestamps.dat";
   public static final String videoPostfix = "Video.mov";
   
   protected final String videoFile;
   protected final String timestampData;
   
   public VideoDataLoggerInterface(File logPath, LogProperties logProperties, String description)
   {
      logProperties.addVideoFile(description);
      logProperties.setInterlaced(description, false);
      String videoFilename = description + videoPostfix;
      logProperties.setVideoFile(description, videoFilename);
      String timestampDataFilename = description + timestampDataPostfix;
      logProperties.setTimestampFile(description, timestampDataFilename);
      
      timestampData = logPath.getAbsolutePath() + File.separator + timestampDataFilename;
      videoFile = logPath.getAbsolutePath() + File.separator + videoFilename;


   }

   public abstract void restart() throws IOException;

   public abstract void close();

   public void removeLogFiles()
   {
      File timestampFile = new File(timestampData);
      if (timestampFile.exists())
      {
         System.out.println("Deleting timestamp data " + timestampData);
         timestampFile.delete();
      }

      File videoFileFile = new File(videoFile);
      if (videoFileFile.exists())
      {
         System.out.println("Deleting video file " + videoFile);
         videoFileFile.delete();
      }
   }

}