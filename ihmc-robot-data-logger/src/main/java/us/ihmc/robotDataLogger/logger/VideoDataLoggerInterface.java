package us.ihmc.robotDataLogger.logger;

import java.io.File;
import java.io.IOException;

import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.Camera;
import us.ihmc.robotDataLogger.LogProperties;

public abstract class VideoDataLoggerInterface
{
   public static final String timestampDataPostfix = "_Timestamps.dat";
   public static final String videoPostfix = "_Video.mov";

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
      newCamera.setName(description);

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
         LogTools.info("Deleting timestamp data " + timestampData);
         timestampFile.delete();
      }

      File videoFileFile = new File(videoFile);
      if (videoFileFile.exists())
      {
         LogTools.info("Deleting video file " + videoFile);
         videoFileFile.delete();
      }
   }

   public abstract void timestampChanged(long newTimestamp);


   /**
    *
    * @return The value of "System.nanotime()" when the last frame was received.
    */
   public abstract long getLastFrameReceivedTimestamp();

}