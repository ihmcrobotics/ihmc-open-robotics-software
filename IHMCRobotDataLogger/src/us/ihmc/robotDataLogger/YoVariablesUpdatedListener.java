package us.ihmc.robotDataLogger;

import java.nio.ByteBuffer;

import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.listeners.ClearLogListener;
import us.ihmc.robotDataLogger.listeners.TimestampListener;

public interface YoVariablesUpdatedListener extends TimestampListener, ClearLogListener
{
   /**
    * 
    * @return true if received variables are updated
    */
   boolean updateYoVariables();

   boolean changesVariables();

   void setShowOverheadView(boolean showOverheadView);

   void start(LogHandshake handshake, YoVariableHandshakeParser handshakeParser);

   /**
    * Called when a timeout is detected on the logger
    */
   void disconnected();

   void setYoVariableClient(YoVariableClient client);

   void receiveTimedOut();

   /**
    * On connect, the return value of this function is send to the server. The server will only send
    * one in n packets to this client
    *  
    * @return Rate of packets to send
    */
   int getDisplayOneInNPackets();



   /**s
    * Data and timestamp is received over the TCP channel. Significant delay can occur depending on the
    * network conditions. Use receivedTimestampOnly for synchronization purposes.
    * 
    * @param timestamp timestamp
    * @param buf data
    */
   void receivedTimestampAndData(long timestamp, ByteBuffer buf);

   boolean executeVariableChangedListeners();
}
