package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.robotDataLogger.rtps.LogProducerDisplay;
import us.ihmc.robotDataLogger.rtps.RTPSDataConsumerParticipant;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerConnection;

public class YoVariableClient
{
   public static final int DEFAULT_TIMEOUT = 15000; //ms
   
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

   /**
    * Use {@link startWithHostSelector}
    * 
    */
   @Deprecated
   public void start()
   {
      start("127.0.0.1", 8008);
//      startWithHostSelector();
   }

   
   /**
    * Start a client for a host selected in the host selector GUI
    */
   public void startWithHostSelector()
   {
      throw new RuntimeException("IMPLEMENT ME");
   }
   
   
   /**
    * Start a logger connecting to a specified host.
    * @param host
    * @param port
    */
   public void start(String host, int port)
   {
      try
      {
      HTTPDataServerConnection connection = HTTPDataServerConnection.connect(host, port);
      start(15000, connection);
      }
      catch(IOException e)
      {
         throw new RuntimeException(e);
      }
   }
   
   

   public void start(int timeout, Announcement announcement) throws IOException
   {
      throw new RuntimeException("TODO: Remove me");
   }
   
   /**
    * Start the logger re-using an already existing HTTPDataServerConnection
    * 
    * This method is used by the logger and the GUI popup to avoid an extra connection. This saves some object allocations on the server side
    * 
    * @param timeout Timeout for requesting resources
    * @param connection An existing HTTPDataServerConnection
    * 
    * @throws IOException 
    */
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
