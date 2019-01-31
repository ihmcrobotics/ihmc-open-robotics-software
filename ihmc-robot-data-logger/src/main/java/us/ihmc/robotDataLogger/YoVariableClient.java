package us.ihmc.robotDataLogger;

import java.io.IOException;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;

import us.ihmc.robotDataLogger.rtps.RTPSDataConsumerParticipant;
import us.ihmc.robotDataLogger.websocket.client.discovery.DataServerDiscoveryClient;
import us.ihmc.robotDataLogger.websocket.client.discovery.DataServerDiscoveryListener;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerConnection;
import us.ihmc.robotDataLogger.rtps.LogProducerDisplay;

public class YoVariableClient
{
   private final YoVariableClientImplementation yoVariableClientImplementation;
   private final LogProducerDisplay.LogSessionFilter[] sessionFilters;

   /**
    * Start a new client while allowing the user to select a desired logging session
    * 
    * @param listener
    * @param filters
    */
   public YoVariableClient(YoVariablesUpdatedListener listener, LogProducerDisplay.LogSessionFilter... filters)
   {
      this.yoVariableClientImplementation = new YoVariableClientImplementation(listener);
      this.sessionFilters = filters;
   }

   /**
    * Connect to an already selected log session
    * @param request
    * @param yoVariablesUpdatedListener
    */
   public YoVariableClient(RTPSDataConsumerParticipant participant, final YoVariablesUpdatedListener yoVariablesUpdatedListener)
   {
      throw new RuntimeException("TODO: Remove me");
   }

   public void start()
   {
      try
      {
         start(15000);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public void start(int timeout) throws IOException
   {
//      Announcement announcement = LogProducerDisplay.getAnnounceRequest(dataConsumerParticipant, sessionFilters);
//      start(timeout, announcement);
      
      CompletableFuture<HTTPDataServerConnection> connectionFuture = new CompletableFuture<HTTPDataServerConnection>();
      
      final DataServerDiscoveryClient discoveryClient = new DataServerDiscoveryClient(new DataServerDiscoveryListener()
      {
         
         @Override
         public void disconnected(HTTPDataServerConnection connection)
         {
            // TODO Auto-generated method stub
            
         }
         
         @Override
         public void connected(HTTPDataServerConnection connection)
         {
            connectionFuture.complete(connection);
         }
      });
      
      discoveryClient.addHost("127.0.0.1", 8008, true);
      
      try
      {
         HTTPDataServerConnection connection = connectionFuture.get();
         start(15000, connection);
      }
      catch (InterruptedException | ExecutionException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
   }

   public void start(int timeout, Announcement announcement) throws IOException
   {
      throw new RuntimeException("TODO: Remove me");
   }
   
   public void start(int timeout, HTTPDataServerConnection connection) throws IOException
   {
      yoVariableClientImplementation.start(timeout, connection);
   }
   
   
   public void reconnect() throws IOException
   {
      yoVariableClientImplementation.reconnect();
   }
   
   public void disconnect()
   {
      yoVariableClientImplementation.disconnect();
   }
   
}
