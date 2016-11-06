package us.ihmc.robotDataLogger.logger;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;

public class LogPropertiesWriter extends LogProperties
{
   private static final long serialVersionUID = -7146049485411068887L;

   private final File file;
   
   public LogPropertiesWriter(File file)
   {
      super();
      this.file = file;
      if(file.exists())
      {
         throw new RuntimeException("Properties file " + file.getAbsolutePath() + " already exists");
      }
      setProperty("version", version);
      
      // Backwards comparability options
      setProperty("video.hasTimebase", "true");
   }
   
   public void store() throws IOException
   {
      FileOutputStream os = new FileOutputStream(file);
      OutputStreamWriter writer = new OutputStreamWriter(os);
      store(writer, "Written by yovariable data logger");
      writer.flush();
      os.getFD().sync();
      writer.close();
   }

}
