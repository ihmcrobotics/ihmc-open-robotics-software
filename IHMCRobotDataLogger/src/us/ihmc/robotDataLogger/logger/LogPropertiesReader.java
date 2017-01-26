package us.ihmc.robotDataLogger.logger;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;

public class LogPropertiesReader extends LogProperties
{
   private static final long serialVersionUID = -2998360732924390734L;

   public LogPropertiesReader(File file)
   {
      super();
      try
      {
         FileReader reader = new FileReader(file);
         load(reader);
         reader.close();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Cannot load properties " + file.getAbsolutePath());
      }
   }
   
}
