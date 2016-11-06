package us.ihmc.robotDataLogger.logger;

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
   public final static double defaultVideoQuality = 0.75;

   private String logDirectory = defaultLogDirectory;
   private double videoQuality = defaultVideoQuality;

   private boolean disableVideo = false;
   
   private boolean flushAggressivelyToDisk = false;

   public static YoVariableLoggerOptions parse(String[] args) throws JSAPException
   {
      SimpleJSAP jsap = new SimpleJSAP("YoVariabeLogger", "Logs YoVariables and video from a robot", new Parameter[] {
            new Switch("disableVideo", 'n', "noVideo", "Disable video recording"),
            new FlaggedOption("logDirectory", JSAP.STRING_PARSER, YoVariableLoggerOptions.defaultLogDirectory, JSAP.NOT_REQUIRED, 'd', "directory",
                  "Directory where to save log files"),
            new FlaggedOption("videoQuality", JSAP.DOUBLE_PARSER, String.valueOf(YoVariableLoggerOptions.defaultVideoQuality), JSAP.NOT_REQUIRED, 'q',
                  "quality", "Video quality"),
            new Switch("flushAggressivelyToDisk", 's', "sync", "Aggressively flush data to disk. Reduces change of data loss but doesn't work on slow platters.") });
      JSAPResult config = jsap.parse(args);
      if (jsap.messagePrinted())
      {
         System.out.println(jsap.getUsage());
         System.out.println(jsap.getHelp());
         System.exit(-1);
      }

      YoVariableLoggerOptions options = new YoVariableLoggerOptions();
      options.setLogDirectory(config.getString("logDirectory"));
      options.setVideoQuality(config.getDouble("videoQuality"));
      options.setDisableVideo(config.getBoolean("disableVideo"));
      options.setFlushAggressivelyToDisk(config.getBoolean("flushAggressivelyToDisk"));

      return options;
   }

   public String getLogDirectory()
   {
      return logDirectory;
   }

   public void setLogDirectory(String logDirectory)
   {
      this.logDirectory = logDirectory;
   }

   public double getVideoQuality()
   {
      return videoQuality;
   }

   public void setVideoQuality(double videoQuality)
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

   public boolean isFlushAggressivelyToDisk()
   {
      return flushAggressivelyToDisk;
   }

   public void setFlushAggressivelyToDisk(boolean flushAggressivelyToDisk)
   {
      this.flushAggressivelyToDisk = flushAggressivelyToDisk;
   }
   
   
}
