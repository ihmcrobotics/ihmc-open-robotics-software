package us.ihmc.robotDataCommunication.logger;

import java.io.File;
import java.io.IOException;

import us.ihmc.robotDataCommunication.logger.util.FFMpeg;
import us.ihmc.robotDataCommunication.logger.util.BMDCapture;
import us.ihmc.robotDataCommunication.logger.util.PipedCommandExecutor;
import us.ihmc.utilities.net.TimestampListener;

public class VideoDataLogger implements TimestampListener
{
   private static final String timestampDataPostfix = "Timestamps.dat";
   private static final String videoPostfix = "Video.mov";
   
   
   /**
    * Make sure to set a progressive mode, otherwise the timestamps will be all wrong!
    */
//   private static final int mode = 11; // 1080p59.94Hz
//   private static final int mode = 9; // 1080p60
   private static final int mode = 14; // 720p59.94Hz
   private static final boolean interlaced = false;
   
   private final PipedCommandExecutor commandExecutor;
   
   public VideoDataLogger(File logPath, LogProperties logProperties, VideoSettings settings, YoVariableLoggerOptions options) throws IOException
   {
      logProperties.setInterlaced(settings.getDescription(), interlaced);
      String videoFilename = settings.getDescription() + videoPostfix;
      logProperties.setVideoFile(settings.getDescription(), videoFilename);
      String timestampDataFilename = settings.getDescription() + timestampDataPostfix;
      logProperties.setTimestampFile(settings.getDescription(), timestampDataFilename);
      
      BMDCapture bmdCapture = new BMDCapture();
      bmdCapture.setMode(mode);
      bmdCapture.setFormat("nut");
      bmdCapture.setFilename("pipe:1");
      bmdCapture.setTimestampData(logPath.getAbsolutePath() + File.separator + timestampDataFilename);
      
    
      FFMpeg avconv = new FFMpeg();
      avconv.setAudioCodec(null);
      avconv.setVideoCodec(options.getVideoCodec());
      avconv.setQuality(options.getVideoQuality());
      avconv.setInputFile("-");
      avconv.setOutputFile(logPath.getAbsolutePath() + File.separator + videoFilename);

      System.out.println();
      System.out.println(avconv.getCommandLine());
      commandExecutor = new PipedCommandExecutor(bmdCapture, avconv);
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
   
}
