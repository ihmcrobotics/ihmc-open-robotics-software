package us.ihmc.robotDataCommunication.logger;

import java.io.File;
import java.io.IOException;

import us.ihmc.communication.net.TimestampListener;
import us.ihmc.robotDataCommunication.logger.util.BMDCapture;
import us.ihmc.robotDataCommunication.logger.util.FFMpeg;
import us.ihmc.robotDataCommunication.logger.util.PipedCommandExecutor;

public class VideoDataLogger implements TimestampListener
{
   private static final String timestampDataPostfix = "Timestamps.dat";
   private static final String videoPostfix = "Video.mov";

   /**
    * Make sure to set a progressive mode, otherwise the timestamps will be all wrong!
    */

   private final PipedCommandExecutor commandExecutor;
   private final String videoFile;
   private final String timestampData;

   public VideoDataLogger(File logPath, LogProperties logProperties, VideoSettings settings, YoVariableLoggerOptions options) throws IOException
   {
      logProperties.addVideoFile(settings.getDescription());
      logProperties.setInterlaced(settings.getDescription(), settings.isInterlaced());
      String videoFilename = settings.getDescription() + videoPostfix;
      logProperties.setVideoFile(settings.getDescription(), videoFilename);
      String timestampDataFilename = settings.getDescription() + timestampDataPostfix;
      logProperties.setTimestampFile(settings.getDescription(), timestampDataFilename);

      BMDCapture bmdCapture = new BMDCapture();
      bmdCapture.setCard(settings.getDevice());
      bmdCapture.setMode(settings.getMode());
      bmdCapture.setVideoIn(settings.getVideoIn());
      bmdCapture.setFormat("nut");
      bmdCapture.setFilename("pipe:1");
      timestampData = logPath.getAbsolutePath() + File.separator + timestampDataFilename;
      bmdCapture.setTimestampData(timestampData);

      FFMpeg avconv = new FFMpeg();
      avconv.setAudioCodec(null);
      avconv.setVideoCodec(options.getVideoCodec());
      avconv.setQuality(options.getVideoQuality());
      avconv.setInputFile("-");
      videoFile = logPath.getAbsolutePath() + File.separator + videoFilename;
      avconv.setOutputFile(videoFile);

      commandExecutor = new PipedCommandExecutor(bmdCapture, avconv);
      commandExecutor.execute();

   }
   
   public void restart() throws IOException
   {
      close();
      removeLogFiles();
      commandExecutor.execute();
   }

   public void timestampChanged(long newTimestamp)
   {
      commandExecutor.writeln(Long.toString(newTimestamp));
   }

   public void close()
   {
      System.out.println("Signalling recorder");
      commandExecutor.writeln("-1");
      commandExecutor.waitFor();
   }

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
