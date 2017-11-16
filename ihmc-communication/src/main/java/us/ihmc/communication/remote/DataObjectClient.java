package us.ihmc.communication.remote;

import java.io.IOException;
import java.net.ConnectException;
import java.net.Socket;

import us.ihmc.commons.thread.ThreadTools;

public class DataObjectClient extends DataObjectTransponder
{
   private String host;
   private int port;

   public DataObjectClient(String host, int port)
   {
      this.host = host;
      this.port = port;
      ThreadTools.startAsDaemon(new ClientConnectionDaemon(), "Data Object Client Conn Daemon");
   }

   public class ClientConnectionDaemon implements Runnable
   {
      private static final int RECONNECTION_PAUSE_DURATION = 100;
      private static final int LOST_CONNECTION_SLEEP_DURATION = 100;

      @Override
      public void run()
      {
         while (true)
         {
            while (!isConnected() && (socket == null))
            {
               tryToConnect();
            }

            sleep(LOST_CONNECTION_SLEEP_DURATION);
         }
      }

      private void tryToConnect()
      {
         try
         {
            connectToSocket(new Socket(host, port));
         }
         catch (ConnectException ce)
         {
            printIfDebug("Problem connecting to data source at " + host + ":" + port + ". Trying again...");
            sleep(RECONNECTION_PAUSE_DURATION);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }

      private void sleep(int duration)
      {
         try
         {
            Thread.sleep(duration);
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }
      }
   }
}
