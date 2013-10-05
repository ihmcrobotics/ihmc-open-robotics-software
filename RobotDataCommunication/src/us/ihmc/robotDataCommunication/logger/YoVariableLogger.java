package us.ihmc.robotDataCommunication.logger;

import java.io.File;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;

import us.ihmc.robotDataCommunication.YoVariableClient;

public class YoVariableLogger
{
   public static final String defaultHost = "192.168.6.202";
   public static final int defaultPort = 5555;
   
   
   private final YoVariableClient client;
   public YoVariableLogger(String robotName, String host, int port, YoVariableLoggerOptions options) throws IOException
   {
      DateFormat dateFormat = new SimpleDateFormat("yyyMMddHHmmss");
      Calendar calendar = Calendar.getInstance();
      String timestamp = dateFormat.format(calendar.getTime());
      File directory = new File(options.getLogDirectory(), robotName + "_" + timestamp);
      if(directory.exists())
      {
         throw new IOException("Directory " + directory.getAbsolutePath() + " already exists");
      }
      if(!directory.mkdir())
      {
         throw new IOException("Cannot create directory " + directory.getAbsolutePath());
      }
      
     
     
      YoVariableLoggerListener logger = new YoVariableLoggerListener(directory, options);
      client = new YoVariableClient(host, port, logger, "");
      client.start();
   }
   
   
   public void waitFor()
   {
      client.waitFor();
   }
   
   
   
   public static void main(String[] args) throws IOException
   {
      new YoVariableLogger("atlas", defaultHost, defaultPort, new YoVariableLoggerOptions());
   }
}
