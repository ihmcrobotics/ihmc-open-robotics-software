package us.ihmc.robotDataCommunication.logger;

import java.io.File;
import java.io.IOException;

import us.ihmc.robotDataCommunication.logger.util.FFMpeg;
import us.ihmc.robotDataCommunication.logger.util.BMDCapture;
import us.ihmc.robotDataCommunication.logger.util.PipedCommandExecutor;
import us.ihmc.utilities.net.TimestampListener;

public class VideoDataLogger implements TimestampListener
{
   private static final String timestampDataFilename = "timestamps.dat";
   private static final String videoFilename = "externalVideo.mov";
   
   
   /**
    * Make sure to set a progressive mode, otherwise the timestamps will be all wrong!
    */
//   private static final int mode = 11; // 1080p59.94Hz
   private static final int mode = 14; // 720p59.94Hz
   private static final boolean interlaced = false;
   
   private final PipedCommandExecutor commandExecutor;
   
   public VideoDataLogger(File logPath, LogProperties logProperties, YoVariableLoggerOptions options) throws IOException
   {
      logProperties.setInterlaced(interlaced);
      logProperties.setVideoFile(videoFilename);
      logProperties.setTimestampFile(timestampDataFilename);
      
      BMDCapture bmdCapture = new BMDCapture(options.getBmdCapturePath());
      bmdCapture.setMode(mode);
      bmdCapture.setFormat("nut");
      bmdCapture.setFilename("pipe:1");
      bmdCapture.setTimestampData(logPath.getAbsolutePath() + File.separator + timestampDataFilename);
      
    
      FFMpeg avconv = new FFMpeg(options.getAvconvPath());
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
   
   public static void main(String[] args)
   {
      BMDCapture bmdCapture = new BMDCapture("bmdcapture");
      bmdCapture.setMode(mode);
      bmdCapture.setFormat("nut");
      bmdCapture.setFilename("pipe:1");
      bmdCapture.setTimestampData(File.separator + timestampDataFilename);
      
    
      FFMpeg avconv = new FFMpeg("avconv");
      avconv.setAudioCodec("copy");
      avconv.setVideoCodec("mjpeg");
      //avconv.setQuality(5);
      avconv.setInputFile("-");
      avconv.setOutputFile(File.separator + videoFilename);
      
      System.out.println(bmdCapture.getCommandLine() + "|" + avconv.getCommandLine());
   }
}
