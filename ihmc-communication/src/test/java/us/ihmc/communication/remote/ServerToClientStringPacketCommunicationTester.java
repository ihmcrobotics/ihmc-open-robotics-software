package us.ihmc.communication.remote;

import us.ihmc.commons.thread.ThreadTools;

class ServerToClientStringPacketCommunicationTester extends AbstractStringPacketCommunicationTester
{
   public ServerToClientStringPacketCommunicationTester(int numberOfPackets, long maxPacketWaitTimeMillis)
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
      ThreadTools.startAsDaemon(new CommsTesterRunnable(StringPacket.getSerialVersionUID(), transponder), "Server Comms Tester (String) Daemon");
   }

   void setupServerStreamingDataConsumers(DataObjectTransponder transponder)
   {
   }
}
