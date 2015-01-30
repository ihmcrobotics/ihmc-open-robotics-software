package us.ihmc.robotDataCommunication.logger;

import java.io.File;
import java.io.IOException;

import us.ihmc.robotDataCommunication.logger.util.BMDCapture;
import us.ihmc.robotDataCommunication.logger.util.FFMpeg;
import us.ihmc.robotDataCommunication.logger.util.PipedCommandExecutor;

public class BlackmagicVideoDataLogger extends VideoDataLoggerInterface
{

   /**
    * Make sure to set a progressive mode, otherwise the timestamps will be all wrong!
    */

   private final PipedCommandExecutor commandExecutor;


   public BlackmagicVideoDataLogger(File logPath, LogProperties logProperties, VideoSettings settings, YoVariableLoggerOptions options) throws IOException
   {
      super(logPath, logProperties, settings.getDescription(), settings.isInterlaced());
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
      bmdCapture.setTimestampData(timestampData);

      FFMpeg avconv = new FFMpeg();
      avconv.setAudioCodec(null);
      avconv.setVideoCodec(options.getVideoCodec());
      avconv.setQuality(options.getVideoQuality());
      avconv.setInputFile("-");
      avconv.setOutputFile(videoFile);

      commandExecutor = new PipedCommandExecutor(bmdCapture, avconv);
      commandExecutor.execute();

   }
   
   /* (non-Javadoc)
    * @see us.ihmc.robotDataCommunication.logger.VideoDataLoggerInterface#restart()
    */
   @Override
   public void restart() throws IOException
   {
      close();
      removeLogFiles();
      commandExecutor.execute();
   }

   /* (non-Javadoc)
    * @see us.ihmc.robotDataCommunication.logger.VideoDataLoggerInterface#timestampChanged(long)
    */
   @Override
   public void timestampChanged(long newTimestamp)
   {
      commandExecutor.writeln(Long.toString(newTimestamp));
   }

   /* (non-Javadoc)
    * @see us.ihmc.robotDataCommunication.logger.VideoDataLoggerInterface#close()
    */
   @Override
   public void close()
   {
      System.out.println("Signalling recorder");
      commandExecutor.writeln("-1");
      commandExecutor.waitFor();
   }



}
