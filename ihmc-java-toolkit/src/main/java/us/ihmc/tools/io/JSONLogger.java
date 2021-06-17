package us.ihmc.tools.io;

import com.fasterxml.jackson.databind.ObjectMapper;
import us.ihmc.log.LogTools;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;

public class JSONLogger<T>
{
   private ObjectMapper objectMapper = null;

   private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
   private static final String logDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;

   String fileName = null;
   private PrintStream printStream = null;

   private boolean isClosed = false;

   public JSONLogger() {
      objectMapper = new ObjectMapper();
   }

   public boolean put(T o) {
      if (fileName == null) { //Null when no object has been written yet
         LogTools.info("Starting JSONLogger instance for " + o.getClass().getSimpleName() + "...");

         fileName = logDirectory + dateFormat.format(new Date()) + "_" + o.getClass().getSimpleName() + "_Log.json";

         try {
            FileOutputStream outputStream = new FileOutputStream(fileName);
            printStream = new PrintStream(outputStream);
         } catch (IOException ex) {
            ex.printStackTrace();
            return false;
         }

         printStream.println("{");
         printStream.println("\"type\": \"" + o.getClass().getSimpleName() + "\",");
         printStream.println("\"data\": [");
      } else {
         printStream.println(",");
      }

      try
      {
         printStream.write(objectMapper.writeValueAsBytes(o));
      }
      catch (IOException ex) {
         ex.printStackTrace();
         return false;
      }

      return true;
   }

   public void close() {
      LogTools.info("Closing " + fileName);

      printStream.println("]\n}");
      printStream.flush();
      printStream.close();

      isClosed = true;
   }

   public boolean isClosed() {
      return isClosed;
   }

   @Override
   public void finalize() {
      if (!isClosed)
         this.close();
   }
}
