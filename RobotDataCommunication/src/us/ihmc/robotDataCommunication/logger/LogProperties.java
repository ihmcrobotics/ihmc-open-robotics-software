package us.ihmc.robotDataCommunication.logger;

import java.util.Properties;

public abstract class LogProperties extends Properties
{
   protected static final String version = "1.0";
   
   private static final long serialVersionUID = -2904334709894113620L;
   
   
   public void setRobotname(String robotName)
   {
      setProperty("general.robotName", robotName);
   }
   
   public String getRobotname()
   {
     return getProperty("general.robotName");
   }
   
   public void setInterlaced(boolean interlaced)
   {
      setProperty("video.interlaced", interlaced?"true":"false");
   }
   
   public boolean getInterlaced()
   {
      return getProperty("video.interlaced").equals("true");
   }
   
   public void setHandshakeFile(String filename)
   {
      setProperty("variables.handshake", filename);
   }
   
   public String getHandshakeFile()
   {
      return getProperty("variables.handshake");
   }
   
   public void setVariableDataFile(String filename)
   {
      setProperty("variables.data", filename);
   }
   
   public String getVariableDataFile()
   {
      return getProperty("variables.data");
   }
   
   public void setVideoFile(String filename)
   {
      setProperty("video.video", filename);
   }
   
   public String getVideoFile()
   {
      return getProperty("video.video");
   }
   
   public void setTimestampFile(String filename)
   {
      setProperty("video.timestamps", filename);
   }
   
   public String getTimestampFile()
   {
      return getProperty("video.timestamps");
   }
}

