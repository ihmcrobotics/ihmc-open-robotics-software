package us.ihmc.robotDataLogger.logger;

import java.io.IOException;
import java.net.InetAddress;
import java.net.NetworkInterface;

import com.martiansoftware.jsap.JSAPException;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.multicastLogDataProtocol.LogUtils;
import us.ihmc.multicastLogDataProtocol.broadcast.AnnounceRequest;
import us.ihmc.multicastLogDataProtocol.broadcast.LogBroadcastListener;
import us.ihmc.multicastLogDataProtocol.broadcast.LogSessionBroadcastClient;

public class YoVariableLoggerDispatcher implements LogBroadcastListener
{
   private final YoVariableLoggerOptions options;

   public YoVariableLoggerDispatcher(YoVariableLoggerOptions options) throws IOException
   {
      this.options = options;
      System.out.println("Starting YoVariableLoggerDispatcher");

      InetAddress myIP = LogUtils.getMyIP(NetworkParameters.getHost(NetworkParameterKeys.logger));
      NetworkInterface iface = NetworkInterface.getByInetAddress(myIP);

      PrintTools.info("Listening on interface " + iface);
      LogSessionBroadcastClient client = new LogSessionBroadcastClient(iface, this);
      client.start();
      System.out.println("Client started, waiting for announcements");
      try
      {
         client.join();
      }
      catch (InterruptedException e)
      {
      }

      System.err.println("Logger has shut down");
   }

   public static void main(String[] args) throws JSAPException, IOException
   {
      YoVariableLoggerOptions options = YoVariableLoggerOptions.parse(args);
      new YoVariableLoggerDispatcher(options);
   }

   @Override
   public synchronized void logSessionCameOnline(AnnounceRequest request)
   {
      System.out.println("New control session came online " + request);
      if (request.isLog())
      {
         System.out.println("Logging sesion " + request);
         try
         {
            new YoVariableLogger(request, options);
            System.out.println("Logging session started");
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      }
   }

   @Override
   public void logSessionWentOffline(AnnounceRequest description)
   {

   }

}
