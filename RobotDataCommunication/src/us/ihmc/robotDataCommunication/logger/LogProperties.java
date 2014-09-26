package us.ihmc.robotDataCommunication.logger;

import java.util.Properties;

public abstract class LogProperties extends Properties
{
   protected static final String version = "1.1";
   
   private static final long serialVersionUID = -2904334709894113620L;
      
   public boolean hasTimebase()
   {
      if(getProperty("video.hasTimebase") != null)
      {
         return true;
      }
      else
      {
         return false;
      }
   }
   
   public void setRobotname(String robotName)
   {
      setProperty("general.robotName", robotName);
   }
   
   public String getRobotname()
   {
     return getProperty("general.robotName");
   }
   
   public String[] getVideoFiles()
   {
      if(getProperty("videoStreams") != null)
      {
         return getProperty("videoStreams").split(",");
      }
      else
      {
//         return new String[] { "video" };
         return new String[] {};
      }
   }
   
   public void addVideoFile(String description)
   {
      if(getProperty("videoStreams") != null)
      {
         setProperty("videoStreams", getProperty("videoStreams") + "," + description);
      }
      else
      {
         setProperty("videoStreams", description);
      }
   }
   
   public void setInterlaced(String description, boolean interlaced)
   {
      setProperty(description + ".interlaced", interlaced?"true":"false");
   }
   
   public boolean getInterlaced(String description)
   {
      return getProperty(description + ".interlaced").equals("true");
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
   
   public void setVideoFile(String description, String filename)
   {
      setProperty(description + ".video", filename);
   }
   
   public String getVideoFile(String description)
   {
      return getProperty(description + ".video");
   }
   
   public void setTimestampFile(String description, String filename)
   {
      setProperty(description + ".timestamps", filename);
   }
   
   public String getTimestampFile(String description)
   {
      return getProperty(description + ".timestamps");
   }
}

