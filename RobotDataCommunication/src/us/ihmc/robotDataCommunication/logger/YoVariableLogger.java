package us.ihmc.robotDataCommunication.logger;

import java.io.File;
import java.io.IOException;
import java.net.SocketTimeoutException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;

import us.ihmc.multicastLogDataProtocol.broadcast.AnnounceRequest;
import us.ihmc.robotDataCommunication.YoVariableClient;

public class YoVariableLogger
{
   public static final long timeout = 5000;

   private final YoVariableClient client;

   public YoVariableLogger(AnnounceRequest request, YoVariableLoggerOptions options) throws IOException
   {
      DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
      Calendar calendar = Calendar.getInstance();
      String timestamp = dateFormat.format(calendar.getTime());
      File directory = new File(options.getLogDirectory(), timestamp + "_" + request.getName());
      if (directory.exists())
      {
         throw new IOException("Directory " + directory.getAbsolutePath() + " already exists");
      }
      if (!directory.mkdir())
      {
         throw new IOException("Cannot create directory " + directory.getAbsolutePath());
      }

      YoVariableLoggerListener logger = new YoVariableLoggerListener(directory, timestamp, request, options);
      boolean showOverheadView = false;
      client = new YoVariableClient(request, logger, "", showOverheadView);
      try
      {
         client.start(timeout);
      }
      catch (SocketTimeoutException e)
      {
         directory.delete();
         throw e;
      }
   }

   public boolean isRunning()
   {
      return client.isRunning();
   }

}
