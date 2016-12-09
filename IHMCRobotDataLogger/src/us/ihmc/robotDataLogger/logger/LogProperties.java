package us.ihmc.robotDataLogger.logger;

import java.util.Properties;

import org.apache.commons.lang3.StringUtils;

public abstract class LogProperties extends Properties
{
   protected static final String version = "2.0";

   private static final long serialVersionUID = -2904334709894113620L;

   public boolean hasTimebase()
   {
      if (getProperty("video.hasTimebase") != null)
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
      if (getProperty("videoStreams") != null)
      {
         return getProperty("videoStreams").split(",");
      }
      else
      {
         //         return new String[] { "video" };
         return new String[] {};
      }
   }
   
   public void removeVideos()
   {
      remove("videoStreams");
   }

   public void addVideoFile(String description)
   {
      if (getProperty("videoStreams") != null)
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
      setProperty(description + ".interlaced", interlaced ? "true" : "false");
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
   
   public void setSummaryFile(String filename)
   {
      setProperty("variables.summary", filename);
   }
   
   public String getSummaryFile()
   {
      return getProperty("variables.summary");
   }

   public void setModelLoaderClass(String clazz)
   {
      setProperty("model.loader", clazz);
   }

   public String getModelLoaderClass()
   {
      return getProperty("model.loader");
   }

   public void setModelPath(String path)
   {
      setProperty("model.path", path);
   }

   public String getModelPath()
   {
      return getProperty("model.path");
   }

   public void setModelName(String name)
   {
      setProperty("model.name", name);
   }

   public String getModelName()
   {
      return getProperty("model.name");
   }
   public void setModelResourceBundlePath(String path)
   {
      setProperty("model.resourceBundle", path);
   }
   
   public String getModelResourceBundlePath()
   {
      return getProperty("model.resourceBundle");
   }
   
   public void setModelResourceDirectories(String[] directories)
   {
      setProperty("model.resourceDirectories", StringUtils.join(directories, ","));
   }
   
   public String[] getModelResourceDirectories()
   {
      return getProperty("model.resourceDirectories").split(",");
   }
   
   public void setVariablesIndexFile(String indexFileName)
   {
      setProperty("variables.index", indexFileName);
   }
   
   public String getVariablesIndexFile()
   {
      return getProperty("variables.index");
   }
   
   public void setCompressed(boolean compressed)
   {
      setProperty("variables.compressed", String.valueOf(compressed));
   }
   
   public boolean getCompressed()
   {
      return Boolean.valueOf(getProperty("variables.compressed"));
   }
   
   public void setTimestamp(String timestamp)
   {
      setProperty("timestamp", timestamp);
   }
   
   public String getTimestamp()
   {
      return getProperty("timestamp");
   }
   
   public void setLogName(String logName)
   {
      setProperty("name", logName);
   }
   
   public String getLogName()
   {
      return getProperty("name");
   }
   
   public boolean isTimestampedIndex()
   {
      return Boolean.valueOf(getProperty("variables.timestamped"));
   }
   
   public void setTimestampedIndex(boolean timestamped)
   {
      setProperty("variables.timestamped", String.valueOf(timestamped));
   }
}
