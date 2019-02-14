package us.ihmc.robotDataLogger;

import java.io.IOException;

import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Interface to control the YoVariableClient.
 * 
 * 
 * @author jesper
 *
 */
public interface YoVariableClientInterface
{

   /**
    * 
    * @return true if the client interface is connected
    */
   boolean isConnected();
   
   /**
    * 
    * @return YoVariableRegistry with debug variables for this instance of the YoVariableClient
    */
   YoVariableRegistry getDebugRegistry();

   /**
    * Broadcast a clear log request for the current session
    * 
    * If no session is available, this request gets silently ignored.
    */
   void sendClearLogRequest();
   
   
   /**
    * Send a command to the server
    * 
    * @param command
    * @param argument
    */
   void sendCommand(DataServerCommand command, int argument);

   
   /**
    * Set the variable update rate for this client
    * 
    * Note: If the controller does not send monotonically increasing timestamps this could result in no received data. 
    * 
    * @param updateRate Desired update rate in milliseconds
    */
   void setVariableUpdateRate(int updateRate);
   
   
   /**
    * Stops the client completely. 
    * 
    * The participant leaves the domain and a reconnect is not possible.
    */
   void stop();
   
   /**
    * Disconnect the current session
    * 
    * Reconnecting to a compatible session is possible
    */
   void disconnect();
   
   /**
    * Reconnect to a compatible session. 
    * @return false if no compatible sessions are found
    * @throws IOException 
    */
   boolean reconnect() throws IOException;

   /**
    * 
    * @return The name of the logging session as reported by the server
    */
   String getServerName();

}
