package us.ihmc.robotDataCommunication.logger;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Parameter;
import com.martiansoftware.jsap.SimpleJSAP;
import com.martiansoftware.jsap.Switch;

public class YoVariableLoggerOptions
{
   public final static String defaultLogDirectory = System.getProperty("user.home") + "/robotLogs";
   public final static String defaultBmdCapturePath ="bin/bmdcapture"; 
   public final static String defaultAvConvPath = "/usr/local/bin/ffmpeg";
   public final static String defaultVideoCodec = "mjpeg";
   public final static int defaultVideoQuality = 5;
   
   
   private String logDirectory = defaultLogDirectory;
   private String bmdCapturePath = defaultBmdCapturePath;
   private String avconvPath = defaultAvConvPath;

   private String videoCodec = defaultVideoCodec;
   private int videoQuality = defaultVideoQuality;

   private boolean disableVideo = false;

   
   public static YoVariableLoggerOptions parse(String[] args) throws JSAPException
   {
      SimpleJSAP jsap = new SimpleJSAP("YoVariabeLogger", "Logs YoVariables and video from a robot", 
            new Parameter[]
            {
               new Switch("disableVideo", 'n', "noVideo", "Disable video recording"),
               new FlaggedOption("logDirectory", JSAP.STRING_PARSER, YoVariableLoggerOptions.defaultLogDirectory, JSAP.NOT_REQUIRED, 'd', "directory", "Directory where to save log files"),
               new FlaggedOption("bmdCapturePath", JSAP.STRING_PARSER, YoVariableLoggerOptions.defaultBmdCapturePath, JSAP.NOT_REQUIRED, 'b', "bmdcapture", "Full path of bmdcapture"),
               new FlaggedOption("avconvPath", JSAP.STRING_PARSER, YoVariableLoggerOptions.defaultAvConvPath, JSAP.NOT_REQUIRED, 'a', "avconv", "Full path of avconv"),
               new FlaggedOption("videoCodec", JSAP.STRING_PARSER, YoVariableLoggerOptions.defaultVideoCodec, JSAP.NOT_REQUIRED, 'c', "codec", "Video codec to use"),
               new FlaggedOption("videoQuality", JSAP.INTEGER_PARSER, String.valueOf(YoVariableLoggerOptions.defaultVideoQuality), JSAP.NOT_REQUIRED, 'q', "quality", "Video quality")
            }
      );
      JSAPResult config = jsap.parse(args);
      if(jsap.messagePrinted()) System.exit(-1);
      
      YoVariableLoggerOptions options = new YoVariableLoggerOptions();
      options.setLogDirectory(config.getString("logDirectory"));
      options.setBmdCapturePath(config.getString("bmdCapturePath"));
      options.setAvconvPath(config.getString("avconvPath"));
      options.setVideoCodec(config.getString("videoCodec"));
      options.setVideoQuality(config.getInt("videoQuality"));
      options.setDisableVideo(config.getBoolean("disableVideo"));
      
      return options;
   }
   
   public String getLogDirectory()
   {
      return logDirectory;
   }

   public String getBmdCapturePath()
   {
      return bmdCapturePath;
   }

   public String getAvconvPath()
   {
      return avconvPath;
   }

   public void setLogDirectory(String logDirectory)
   {
      this.logDirectory = logDirectory;
   }

   public void setBmdCapturePath(String bmdCapturePath)
   {
      this.bmdCapturePath = bmdCapturePath;
   }

   public void setAvconvPath(String avconvPath)
   {
      this.avconvPath = avconvPath;
   }

   public String getVideoCodec()
   {
      return videoCodec;
   }

   public int getVideoQuality()
   {
      return videoQuality;
   }

   public void setVideoCodec(String videoCodec)
   {
      this.videoCodec = videoCodec;
   }

   public void setVideoQuality(int videoQuality)
   {
      this.videoQuality = videoQuality;
   }

   public boolean getDisableVideo()
   {
      return disableVideo;
   }

   public void setDisableVideo(boolean disableVideo)
   {
      this.disableVideo = disableVideo;
   }
}
