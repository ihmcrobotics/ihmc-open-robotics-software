package us.ihmc.robotDataCommunication.logger.util;

public class AVConv implements ExternalProgram
{
   private static final String avconvBinary = "ffmpeg";

   private final String path;
   
   private String inputFile;
   private String outputFile;

   private String videoCodec;
   private String audioCodec;

   private int quality = -1;
   
   private boolean showinfo = false;

   public AVConv()
   {
      this(avconvBinary);
   }
   
   public AVConv(String path)
   {
      this.path = path;
   }

   public void setInputFile(String file)
   {
      inputFile = file;
   }

   public void readFromStdin()
   {
      setInputFile("-");
   }

   public void setOutputFile(String outputFile)
   {
      this.outputFile = outputFile;
   }
   
   public void writeToStdout()
   {
      setOutputFile("-");
   }

   public void setVideoCodec(String videoCodec)
   {
      this.videoCodec = videoCodec;
   }

   public void setAudioCodec(String audioCodec)
   {
      this.audioCodec = audioCodec;
   }

   public void setShowinfo(boolean showinfo)
   {
      this.showinfo = showinfo;
   }
   
   public void setQuality(int quality)
   {
      this.quality = quality;
   }
   
   private void appendCmdOption(StringBuilder cmd, String... args)
   {
      for (String arg : args)
      {
         cmd.append(" ");
         cmd.append(arg);
      }
   }

   public String getCommandLine()
   {
      StringBuilder cmd = new StringBuilder();
      cmd.append(path);

      if (inputFile != null)
      {
         appendCmdOption(cmd, "-i", inputFile);
      }

      if (videoCodec != null)
      {
         appendCmdOption(cmd, "-vcodec", videoCodec);
      }

      if (audioCodec != null)
      {
         appendCmdOption(cmd, "-acodec", audioCodec);
      }
      else
      {
         appendCmdOption(cmd, "-an");
      }
      
      if(quality != -1)
      {
         appendCmdOption(cmd, "-q:v", String.valueOf(quality));
      }

      if (showinfo)
      {
         appendCmdOption(cmd, "-vf", "showinfo");
      }

      if (outputFile != null)
      {
    	 cmd.append(" ");
         cmd.append(outputFile);
      }

      return cmd.toString();
   }
}
