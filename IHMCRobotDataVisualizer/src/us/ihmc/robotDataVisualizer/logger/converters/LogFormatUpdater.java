package us.ihmc.robotDataVisualizer.logger.converters;

import java.io.File;
import java.io.IOException;

import us.ihmc.robotDataLogger.LogProperties;

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
         
         if (properties.getVariables().getCompressed() && !properties.getVariables().getTimestamped())
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
