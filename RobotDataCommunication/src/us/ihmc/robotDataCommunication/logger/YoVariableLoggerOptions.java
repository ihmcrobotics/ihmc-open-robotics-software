package us.ihmc.robotDataCommunication.logger;

import java.util.ArrayList;
import java.util.Arrays;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPException;
import com.martiansoftware.jsap.JSAPResult;
import com.martiansoftware.jsap.Parameter;
import com.martiansoftware.jsap.SimpleJSAP;
import com.martiansoftware.jsap.Switch;
import com.martiansoftware.jsap.UnflaggedOption;

public class YoVariableLoggerOptions
{
   public final static String defaultLogDirectory = System.getProperty("user.home") + "/robotLogs";
   public final static String defaultVideoCodec = "mjpeg";
   public final static int defaultVideoQuality = 5;

   public final static String defaultConfigurationFile = "cameras.yaml";

   private String logDirectory = defaultLogDirectory;
   private String videoCodec = defaultVideoCodec;
   private int videoQuality = defaultVideoQuality;

   private boolean enableCookieJar = false;
   private String cookieJarUser = "";
   private String cookieJarHost = "";
   private String cookieJarRemoteDirectory = "";
   private boolean disableVideo = false;
   private String configurationFile = "";

   public static YoVariableLoggerOptions parse(String[] args) throws JSAPException
   {
      SimpleJSAP jsap = new SimpleJSAP("YoVariabeLogger", "Logs YoVariables and video from a robot", new Parameter[] {
            new Switch("disableVideo", 'n', "noVideo", "Disable video recording"),
            new FlaggedOption("logDirectory", JSAP.STRING_PARSER, YoVariableLoggerOptions.defaultLogDirectory, JSAP.NOT_REQUIRED, 'd', "directory",
                  "Directory where to save log files"),
            new FlaggedOption("videoCodec", JSAP.STRING_PARSER, YoVariableLoggerOptions.defaultVideoCodec, JSAP.NOT_REQUIRED, 'c', "codec",
                  "Video codec to use"),
            new FlaggedOption("configurationFile", JSAP.STRING_PARSER, YoVariableLoggerOptions.defaultConfigurationFile, JSAP.NOT_REQUIRED, 'f', "config",
                  "Camera configuration file"),
            new FlaggedOption("videoQuality", JSAP.INTEGER_PARSER, String.valueOf(YoVariableLoggerOptions.defaultVideoQuality), JSAP.NOT_REQUIRED, 'q',
                  "quality", "Video quality") });
      JSAPResult config = jsap.parse(args);
      if (jsap.messagePrinted())
      {
         System.out.println(jsap.getUsage());
         System.out.println(jsap.getHelp());
         System.exit(-1);
      }

      YoVariableLoggerOptions options = new YoVariableLoggerOptions();
      options.setLogDirectory(config.getString("logDirectory"));
      options.setVideoCodec(config.getString("videoCodec"));
      options.setVideoQuality(config.getInt("videoQuality"));
      options.setDisableVideo(config.getBoolean("disableVideo"));
      options.setConfigurationFile(config.getString("configurationFile"));

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

   public void setEnableCookieJar(boolean enableCookieJar)
   {
      this.enableCookieJar = enableCookieJar;
   }

   public void setCookieJarUser(String cookieJarUser)
   {
      this.cookieJarUser = cookieJarUser;
   }

   public void setCookieJarHost(String cookieJarHost)
   {
      this.cookieJarHost = cookieJarHost;
   }

   public void setCookieJarRemoteDirectory(String cookieJarRemoteDirectory)
   {
      this.cookieJarRemoteDirectory = cookieJarRemoteDirectory;
   }

   public boolean isEnableCookieJar()
   {
      return enableCookieJar;
   }

   public String getCookieJarUser()
   {
      return cookieJarUser;
   }

   public String getCookieJarHost()
   {
      return cookieJarHost;
   }

   public String getCookieJarRemoteDirectory()
   {
      return cookieJarRemoteDirectory;
   }

   public String getConfigurationFile()
   {
      return configurationFile;
   }

   public void setConfigurationFile(String configurationFile)
   {
      this.configurationFile = configurationFile;
   }
}
