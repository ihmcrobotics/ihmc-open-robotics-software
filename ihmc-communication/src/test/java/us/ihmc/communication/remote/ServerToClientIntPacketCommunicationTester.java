package us.ihmc.communication.remote;

import us.ihmc.commons.thread.ThreadTools;

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
