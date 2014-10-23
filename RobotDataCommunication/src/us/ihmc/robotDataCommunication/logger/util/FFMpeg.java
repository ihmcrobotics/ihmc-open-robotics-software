package us.ihmc.robotDataCommunication.logger.util;

import us.ihmc.utilities.operatingSystem.OperatingSystem;

public class FFMpeg implements ExternalProgram
{
   /*
    * LibAV is broken with respect to timestamps, use ffmpeg version 2.0.2.
    */
   private static final String builtinFFMpeg;
   static
   {
      String temp = ExternalProgramHelpers.extractExternalProgram(FFMpeg.class.getResource(
            "bin/" + ExternalProgramHelpers.getOSNameAsString() + "/ffmpeg" + ExternalProgramHelpers.getExecutableExtension()));
      if (ExternalProgramHelpers.getOS() == OperatingSystem.WINDOWS)
      {
         // Windows doesn't handle the first slash in the absolute path string: "/C:/..."
         builtinFFMpeg = temp.substring(1);
      }
      else
      {
         builtinFFMpeg = temp;
      }
   }

   private final String path;

   private String inputFile;
   private String outputFile;

   private String videoCodec;
   private String audioCodec;

   private int quality = -1;
   private double startTime = -1;
   private double endTime = -1;

   private boolean showinfo = false;
   private boolean enableExperimentalCodecs = false;

   public FFMpeg()
   {
      this.path = builtinFFMpeg;
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

      appendCmdOption(cmd, "-y");
      if (startTime > 0)
      {
         appendCmdOption(cmd, "-ss", String.valueOf(startTime));
      }

      if (inputFile != null)
      {
         appendCmdOption(cmd, "-i", inputFile);
      }

      if (endTime > 0)
      {
         double duration = endTime;
         if (startTime > 0)
         {
            duration -= startTime;
         }
         appendCmdOption(cmd, "-t", String.valueOf(duration));
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

      if (enableExperimentalCodecs)
      {
         appendCmdOption(cmd, "-strict", "-2");
      }

      if (quality != -1)
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

   public void setStarttime(double startTime)
   {
      this.startTime = startTime;
   }

   public void setEndtime(double endTime)
   {
      this.endTime = endTime;
   }

   public void enableExperimentalCodecs(boolean b)
   {
      this.enableExperimentalCodecs = b;
   }
}
