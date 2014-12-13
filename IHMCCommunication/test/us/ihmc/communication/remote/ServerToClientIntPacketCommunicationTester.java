package us.ihmc.communication.remote;

import us.ihmc.communication.remote.DataObjectTransponder;
import us.ihmc.communication.remote.IntegerPacket;
import us.ihmc.utilities.ThreadTools;

class ServerToClientIntPacketCommunicationTester extends AbstractIntegerPacketCommunicationTester
{
   public ServerToClientIntPacketCommunicationTester(int numberOfPackets, long maxPacketWaitTimeMillis)
   {
      super(numberOfPackets, maxPacketWaitTimeMillis);
   }

   void setupClientDaemons(DataObjectTransponder transponder)
   {
   }

   void setupClientStreamingDataConsumers(DataObjectTransponder transponder)
   {
      transponder.addStreamingDataConsumer(streamingDataConsumer);
   }

   void setupServerDaemons(DataObjectTransponder transponder)
   {
      ThreadTools.startAsDaemon(new CommsTesterRunnable(IntegerPacket.getSerialVersionUID(), transponder), "Server Comms Tester (Int) Daemon");
   }

   void setupServerStreamingDataConsumers(DataObjectTransponder transponder)
   {
   }
}
