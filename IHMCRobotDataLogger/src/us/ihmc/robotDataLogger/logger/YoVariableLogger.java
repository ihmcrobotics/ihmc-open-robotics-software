package us.ihmc.robotDataLogger.logger;

import java.io.File;
import java.io.IOException;
import java.net.SocketTimeoutException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;

import us.ihmc.multicastLogDataProtocol.broadcast.AnnounceRequest;
import us.ihmc.robotDataLogger.YoVariableClient;

public class YoVariableLogger
{
   public static final long timeout = 5000;

   private final YoVariableClient client;

   public YoVariableLogger(AnnounceRequest request, YoVariableLoggerOptions options) throws IOException
   {
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

      YoVariableLoggerListener logger = new YoVariableLoggerListener(tempDirectory, finalDirectory, timestamp, request, options);
      client = new YoVariableClient(request, logger, "");

      try
      {
         client.start(timeout);
      }
      catch (SocketTimeoutException e)
      {
         finalDirectory.delete();
         throw e;
      }
   }

   public boolean isRunning()
   {
      return client.isRunning();
   }

}
