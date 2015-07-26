package us.ihmc.robotDataCommunication;

import java.nio.ByteBuffer;

import us.ihmc.multicastLogDataProtocol.control.LogHandshake;

public interface YoVariablesUpdatedListener
{
   boolean populateRegistry();

   boolean changesVariables();

   void setShowOverheadView(boolean showOverheadView);

   void start(LogHandshake handshake, YoVariableHandshakeParser handshakeParser);

   void disconnected();

   void setYoVariableClient(YoVariableClient client);

   void receiveTimedOut();

   int getDisplayOneInNPackets();

   void receivedTimestampOnly(long timestamp);

   void receivedTimestampAndData(long timestamp, ByteBuffer buf);

   void clearLog();

   boolean executeVariableChangedListeners();
}
