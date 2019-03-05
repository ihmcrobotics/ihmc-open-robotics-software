package us.ihmc.robotDataLogger;

import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.interfaces.CommandListener;
import us.ihmc.robotDataLogger.listeners.TimestampListener;

public interface YoVariablesUpdatedListener extends TimestampListener, CommandListener
{
   /**
    * 
    * @return true if received variables are updated
    */
   boolean updateYoVariables();

   boolean changesVariables();

   void setShowOverheadView(boolean showOverheadView);

   void start(YoVariableClientInterface yoVariableClientInterface, LogHandshake handshake, YoVariableHandshakeParser handshakeParser);

   /**
    * Called when a timeout is detected on the logger.
    * 
    * A reconnect is possible after being disconnected
    * 
    */
   void disconnected();

   /**
    * Data and timestamp is received over the TCP channel. Significant delay can occur depending on the
    * network conditions. Use receivedTimestampOnly for synchronization purposes.
    * 
    * @param timestamp timestamp
    */
   void receivedTimestampAndData(long timestamp);

   /**
    * Gets called when the client is connected and the first data has been received.
    */
   void connected();
}
