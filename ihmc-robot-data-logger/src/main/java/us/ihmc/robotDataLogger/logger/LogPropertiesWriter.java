package us.ihmc.robotDataLogger.logger;

import java.io.File;
import java.io.IOException;

import us.ihmc.idl.serializers.extra.PropertiesSerializer;
import us.ihmc.robotDataLogger.LogProperties;
import us.ihmc.robotDataLogger.LogPropertiesPubSubType;

public class LogPropertiesWriter extends LogProperties
{
   private final static String version = "3.0";
   private final File file;
   
   public LogPropertiesWriter(File file)
   {
      super();
      this.file = file;
      if(file.exists())
      {
         throw new RuntimeException("Properties file " + file.getAbsolutePath() + " already exists");
      }
      setVersion(version);
      // Backwards comparability options
      getVideo().setHasTimebase(true);
   }
   
   public void store() throws IOException
   {
      PropertiesSerializer<LogProperties> serializer = new PropertiesSerializer<>(new LogPropertiesPubSubType());
      serializer.serialize(file, this);
   }

}
