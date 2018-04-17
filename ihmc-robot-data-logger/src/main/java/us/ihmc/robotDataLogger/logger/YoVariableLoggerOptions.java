package us.ihmc.robotDataLogger.logger;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Parameter;
import com.martiansoftware.jsap.SimpleJSAP;
import com.martiansoftware.jsap.Switch;

import us.ihmc.javadecklink.Capture.CodecID;

public class YoVariableLoggerOptions
{
   public final static String defaultLogDirectory = System.getProperty("user.home") + "/robotLogs";
   
   public final static CodecID defaultCodec = CodecID.AV_CODEC_ID_MJPEG;
   public final static double defaultVideoQuality = 0.85;
   public final static int defaultCRF = 23;

   private String logDirectory = defaultLogDirectory;
   
   private CodecID videoCodec;
   private int crf;
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
                  "quality", "Video quality for MJPEG"),
            new FlaggedOption("videoCodec", JSAP.STRING_PARSER, String.valueOf(defaultCodec), JSAP.NOT_REQUIRED, 'c', "codec", "Desired video codec. AV_CODEC_ID_H264 or AV_CODEC_ID_MJPEG"),
            new FlaggedOption("crf", JSAP.INTEGER_PARSER, String.valueOf(defaultCRF), JSAP.NOT_REQUIRED, 'r', "crf", "CRF (Constant rate factor) for H264. 0-51, 0 is lossless. Sane values are 18 to 28."),
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
      options.setVideoCodec(CodecID.valueOf(config.getString("videoCodec")));
      options.setCrf(config.getInt("crf"));
      
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

   public CodecID getVideoCodec()
   {
      return videoCodec;
   }

   public void setVideoCodec(CodecID videoCodec)
   {
      this.videoCodec = videoCodec;
   }

   public int getCrf()
   {
      return crf;
   }

   public void setCrf(int crf)
   {
      this.crf = crf;
   }
   
   
   
   
}
