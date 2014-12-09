package us.ihmc.robotDataCommunication.logger.util;

import us.ihmc.robotDataCommunication.logger.VideoIn;

public class BMDCapture implements ExternalProgram
{
   private static final String bmdCapture = ExternalProgramHelpers.extractExternalProgram(BMDCapture.class.getResource(
         "bin/" + ExternalProgramHelpers.getOSNameAsString() + "/bmdcapture" + ExternalProgramHelpers.getExecutableExtension()));
   
   private final String path;
   
   
   
   public enum AudioIn
   {
      ANALOG(1),
      EMBEDDED(2),
      DIGITAL(3);
      
      int command;
      private AudioIn(int command)
      {
         this.command = command;
      }
   }
   
   private int mode = -1;
   private String filename;
   private String format;
   private int channels = -1;
   private int depth = -1;
   private int pixelFormat = -1;
   private int memLimit = -1;
   private int card = -1;
   private AudioIn audioIn;
   private VideoIn videoIn;
   private String timestampData;
   
   
   
   /**
    * "    -v                   Be verbose (report each 25 frames)\n"
        "    -f <filename>        Filename raw video will be written to\n"
        "    -F <format>          Define the file format to be used\n"
        "    -c <channels>        Audio Channels (2, 8 or 16 - default is 2)\n"
        "    -s <depth>           Audio Sample Depth (16 or 32 - default is 16)\n"
        "    -p <pixel>           PixelFormat Depth (8 or 10 - default is 8)\n"
        "    -n <frames>          Number of frames to capture (default is unlimited)\n"
        "    -M <memlimit>        Maximum queue size in GB (default is 1 GB)\n"
        "    -C <num>             number of card to be used\n"
        "    -S <serial_device>   data input serial\n"
        "    -A <audio-in>        Audio input:\n"
        "                         1: Analog (RCA or XLR)\n"
        "                         2: Embedded Audio (HDMI/SDI)\n"
        "                         3: Digital Audio (AES/EBU)\n"
        "    -V <video-in>        Video input:\n"
        "                         1: Composite\n"
        "                         2: Component\n"
        "                         3: HDMI\n"
        "                         4: SDI\n"
        "                         5: Optical SDI\n"
        "                         6: S-Video\n"
        "   -o <filename>       Write timestamps read from stdin + frame timestamp to <filename>\n"
        
    * @param path were the bmdcapture executable is located
    */
   
   public BMDCapture()
   {
      this.path = bmdCapture;
   }
   
   
   
   public void setMode(int mode)
   {
      this.mode = mode;
   }



   public void setFilename(String filename)
   {
      this.filename = filename;
   }



   public void setFormat(String format)
   {
      this.format = format;
   }



   public void setChannels(int channels)
   {
      this.channels = channels;
   }



   public void setDepth(int depth)
   {
      this.depth = depth;
   }



   public void setPixelFormat(int pixelFormat)
   {
      this.pixelFormat = pixelFormat;
   }



   public void setMemLimit(int memLimit)
   {
      this.memLimit = memLimit;
   }



   public void setCard(int card)
   {
      this.card = card;
   }



   public void setAudioIn(AudioIn audioIn)
   {
      this.audioIn = audioIn;
   }



   public void setVideoIn(VideoIn videoIn)
   {
      this.videoIn = videoIn;
   }



   public void setTimestampData(String timestampData)
   {
      this.timestampData = timestampData;
   }



   private void appendCmdOption(StringBuilder cmd, String sw, int val)
   {
      cmd.append(" ");
      cmd.append(sw);
      cmd.append(" ");
      cmd.append(val);
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
      
      if(mode != -1)
      {
         appendCmdOption(cmd, "-m", mode);
      }
      
      if(filename != null)
      {
         appendCmdOption(cmd, "-f", filename);
      }
      
      if(format != null)
      {
         appendCmdOption(cmd, "-F", format);
      }
      
      if(channels != -1)
      {
         appendCmdOption(cmd, "-c", channels);
      }
      
      if(depth != -1)
      {
         appendCmdOption(cmd, "-d", depth);
      }
      
      if(pixelFormat != -1)
      {
         appendCmdOption(cmd, "-p", pixelFormat);
      }
      
      if(memLimit != -1)
      {
         appendCmdOption(cmd, "-M", memLimit);
      }
      
      if(card != -1)
      {
         appendCmdOption(cmd, "-C", card);
      }
      
      if(audioIn != null)
      {
         appendCmdOption(cmd, "-A", audioIn.command);
      }
      
      if(videoIn != null)
      {
         appendCmdOption(cmd, "-V", videoIn.command);
      }
      
      if(timestampData != null)
      {
         appendCmdOption(cmd, "-o", timestampData);
      }
      
      return cmd.toString();
   }
   
}
