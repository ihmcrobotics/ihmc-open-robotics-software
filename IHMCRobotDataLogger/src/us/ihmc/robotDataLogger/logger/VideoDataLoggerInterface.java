package us.ihmc.robotDataLogger.logger;

import java.io.File;
import java.io.IOException;

import us.ihmc.robotDataLogger.Camera;
import us.ihmc.robotDataLogger.LogProperties;

public abstract class VideoDataLoggerInterface
{
   public static final String timestampDataPostfix = "Timestamps.dat";
   public static final String videoPostfix = "Video.mov";
   
   protected final String videoFile;
   protected final String timestampData;
   
   public VideoDataLoggerInterface(File logPath, LogProperties logProperties, String description)
   {
      Camera newCamera = logProperties.getCameras().add();
      
      newCamera.setVideoFile(description);
      newCamera.setInterlaced(false);
      String videoFilename = description + videoPostfix;
      newCamera.setVideoFile(videoFilename);
      String timestampDataFilename = description + timestampDataPostfix;
      newCamera.setTimestampFile(timestampDataFilename);
      
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

   public abstract void timestampChanged(long newTimestamp);

}