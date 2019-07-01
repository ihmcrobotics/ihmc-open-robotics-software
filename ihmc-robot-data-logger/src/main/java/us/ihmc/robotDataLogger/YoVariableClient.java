package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.gui.DataServerSelectorGUI;
import us.ihmc.robotDataLogger.rtps.LogProducerDisplay;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerConnection;

/**
 * Main entry point to write a client to the data server 
 * 
 * @author Jesper Smith
 *
 */
public class YoVariableClient
{
   public static final int DEFAULT_TIMEOUT = 25000; //ms
   
   private final YoVariableClientImplementation yoVariableClientImplementation;

   /**
    * LogSessionFilters are not implemented anymore. 
    * 
    * Advised is to set listenForBroadcasts to false and manually set the IP of the target
    * 
    */
   @Deprecated
   public YoVariableClient(YoVariablesUpdatedListener listener, LogProducerDisplay.LogSessionFilter... filters)
   {
      this(listener);
   }
   
   
   /**
    * Start a new client while allowing the user to select a desired logging session
    * 
    * @param listener
    */
   public YoVariableClient(YoVariablesUpdatedListener listener)
   {
      this.yoVariableClientImplementation = new YoVariableClientImplementation(listener);
   }

   /**
    * Use {@link startWithHostSelector}
    * 
    */
   @Deprecated
   public void start()
   {
      startWithHostSelector();
   }

   /**
    * Start a client for a host selected in the host selector GUI
    */
   public void startWithHostSelector()
   {
      startWithHostSelector(true);
   }
   
   /**
    * Start a client for a host selected in the host selector GUI
    * @param enableAutoDiscovery If true, the client will add hosts broadcasting their existence.
    */
   public void startWithHostSelector(boolean enableAutoDiscovery)
   {
      DataServerSelectorGUI selector = new DataServerSelectorGUI(true);
      HTTPDataServerConnection connection = selector.select();
      if(connection != null)
      {
         try
         {
            start(DEFAULT_TIMEOUT, connection);
         }
         catch (IOException e)
         {
            e.printStackTrace();
            System.exit(0);
         }
      }
      else
      {
         LogTools.warn("No host selected. Shutting down.");
         System.exit(0);
      }
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
      start(DEFAULT_TIMEOUT, connection);
      }
      catch(IOException e)
      {
         throw new RuntimeException(e);
      }
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
   
   
   /**
    * Reconnect to the same session.
    * 
    * This will work as long as the IP, port, controller name and complete variable registry match.
    * 
    * @throws IOException
    */
   public void reconnect() throws IOException
   {
      yoVariableClientImplementation.reconnect();
   }
   
   /**
    * Disconnect and cleanup
    */
   public void disconnect()
   {
      yoVariableClientImplementation.disconnect();
   }

}
