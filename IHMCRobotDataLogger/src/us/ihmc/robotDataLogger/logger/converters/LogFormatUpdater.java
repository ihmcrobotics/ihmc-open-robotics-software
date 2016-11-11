package us.ihmc.robotDataLogger.logger.converters;

import java.io.File;
import java.io.IOException;

import us.ihmc.robotDataLogger.logger.LogProperties;
import us.ihmc.robotDataLogger.logger.converters.ModelAttacher.LogModels;

public class LogFormatUpdater
{
   public static void updateLogs(File directory, LogProperties properties)
   {
      try
      {
//         if(properties.getModelLoaderClass() == null)
//         {
//            LogModels model = ModelAttacher.chooseModel(directory);
//            ModelAttacher.addModel(directory, properties, model);
//         }
         
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
