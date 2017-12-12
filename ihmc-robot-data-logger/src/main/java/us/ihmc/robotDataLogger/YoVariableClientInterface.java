package us.ihmc.robotDataLogger;

import java.io.IOException;

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
