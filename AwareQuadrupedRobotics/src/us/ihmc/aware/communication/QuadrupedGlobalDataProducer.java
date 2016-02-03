package us.ihmc.aware.communication;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.streamingData.GlobalDataProducer;

public class QuadrupedGlobalDataProducer extends GlobalDataProducer
{
   public QuadrupedGlobalDataProducer(PacketCommunicator communicator)
   {
      super(communicator);
   }
}
