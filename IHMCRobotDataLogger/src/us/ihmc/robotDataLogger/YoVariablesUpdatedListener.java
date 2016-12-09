package us.ihmc.robotDataLogger;

import java.nio.ByteBuffer;

import us.ihmc.multicastLogDataProtocol.control.LogHandshake;

public interface YoVariablesUpdatedListener
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

   /**
    * Called when an UDP packet with the timestamp is received. The timestamps are send
    * over UDP for synchronization purposes. Timestamps are published directly from the realtime
    * thread and should have minimum delay
    * 
    * @param timestamp
    */
   void receivedTimestampOnly(long timestamp);

   /**s
    * Data and timestamp is received over the TCP channel. Significant delay can occur depending on the
    * network conditions. Use receivedTimestampOnly for synchronization purposes.
    * 
    * @param timestamp timestamp
    * @param buf data
    */
   void receivedTimestampAndData(long timestamp, ByteBuffer buf);

   /**
    * Broadcast to all clients to clear the log and start recording anew.
    * 
    */
   void clearLog();

   boolean executeVariableChangedListeners();
}
