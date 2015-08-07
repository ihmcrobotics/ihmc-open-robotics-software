package us.ihmc.communication.remote;

import us.ihmc.tools.thread.ThreadTools;

class ClientToServerStringPacketCommunicationTester extends AbstractStringPacketCommunicationTester
{
   public ClientToServerStringPacketCommunicationTester(int numberOfPackets, long maxPacketWaitTimeMillis)
   {
      super(numberOfPackets, maxPacketWaitTimeMillis);
   }

   void setupClientStreamingDataConsumers(DataObjectTransponder transponder)
   {
   }

   void setupClientDaemons(DataObjectTransponder transponder)
   {
      ThreadTools.startAsDaemon(new CommsTesterRunnable(StringPacket.getSerialVersionUID(), transponder), "Client Comms Tester (String) Daemon");
   }

   void setupServerStreamingDataConsumers(DataObjectTransponder transponder)
   {
      transponder.addStreamingDataConsumer(streamingDataConsumer);
   }

   void setupServerDaemons(DataObjectTransponder transponder)
   {
   }
}
