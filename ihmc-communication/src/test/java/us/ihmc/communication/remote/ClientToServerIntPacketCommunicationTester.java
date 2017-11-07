package us.ihmc.communication.remote;

import us.ihmc.commons.thread.ThreadTools;

class ClientToServerIntPacketCommunicationTester extends AbstractIntegerPacketCommunicationTester
{
   public ClientToServerIntPacketCommunicationTester(int numberOfPackets, long maxPacketWaitTimeMillis)
   {
      super(numberOfPackets, maxPacketWaitTimeMillis);
   }

   void setupClientStreamingDataConsumers(DataObjectTransponder transponder)
   {
   }

   void setupClientDaemons(DataObjectTransponder transponder)
   {
      ThreadTools.startAsDaemon(new CommsTesterRunnable(IntegerPacket.getSerialVersionUID(), transponder), "Client Comms Tester (Int) Daemon");
   }

   void setupServerStreamingDataConsumers(DataObjectTransponder transponder)
   {
      transponder.addStreamingDataConsumer(streamingDataConsumer);
   }

   void setupServerDaemons(DataObjectTransponder transponder)
   {
   }
}
