package us.ihmc.robotDataCommunication.logger.converters;

import java.io.File;
import java.io.IOException;

import us.ihmc.robotDataCommunication.logger.LogProperties;

public class LogFormatUpdater
{
   public static void updateLogs(File directory, LogProperties properties)
   {
      try
      {
         if (properties.getCompressed() && !properties.isTimestampedIndex())
         {
            LogTimeStampedIndexGenerator.convert(directory, properties);
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException("Cannot convert log file", e);
      }
   }
}
