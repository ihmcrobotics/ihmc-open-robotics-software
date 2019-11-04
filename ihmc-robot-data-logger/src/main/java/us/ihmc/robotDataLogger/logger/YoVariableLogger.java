package us.ihmc.robotDataLogger.logger;

import java.io.File;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.function.Consumer;

import us.ihmc.robotDataLogger.Announcement;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerConnection;

public class YoVariableLogger
{
   // changed to a 10s timeout for camp lejeune demo
   public static final int timeout = 10000; // 2500;

   private final YoVariableClient client;

   public YoVariableLogger(HTTPDataServerConnection connection, YoVariableLoggerOptions options, Consumer<Announcement> doneListener) throws IOException
   {
      Announcement request = connection.getAnnouncement();
      
      DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
      Calendar calendar = Calendar.getInstance();
      String timestamp = dateFormat.format(calendar.getTime());
      
      File tempDirectory = new File(options.getLogDirectory(), "." + timestamp + "_" + request.getName());
      
      File finalDirectory = new File(options.getLogDirectory(), timestamp + "_" + request.getName());
      if (finalDirectory.exists())
      {
         throw new IOException("Directory " + finalDirectory.getAbsolutePath() + " already exists");
      }
      
      if (tempDirectory.exists())
      {
         throw new IOException("Temp directory " + finalDirectory.getAbsolutePath() + " already exists");
      }
      if (!tempDirectory.mkdir())
      {
         throw new IOException("Cannot create directory " + finalDirectory.getAbsolutePath());
      }

      YoVariableLoggerListener logger = new YoVariableLoggerListener(tempDirectory, finalDirectory, timestamp, request, options, doneListener);
      client = new YoVariableClient(logger);

      try
      {
         client.start(timeout, connection);
      }
      catch (IOException e)
      {
         finalDirectory.delete();
         throw e;
      }
   }
}
