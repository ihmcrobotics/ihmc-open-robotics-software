package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.walking.AutomaticManipulationAbortPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.ManipulationAbortedStatus;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;

public class AutomaticManipulationAbortCommunicator implements PacketConsumer<AutomaticManipulationAbortPacket>
{
   private final HumanoidGlobalDataProducer globalDataProducer;
   private final AtomicBoolean hasReceivedNewPacket = new AtomicBoolean(false);
   private final AtomicBoolean enableAutomaticManipulationAbort = new AtomicBoolean(false);

   public AutomaticManipulationAbortCommunicator(HumanoidGlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;
   }

   public boolean checkForNewInformation()
   {
      return hasReceivedNewPacket.getAndSet(false);
   }

   public boolean isAutomaticManipulationAbortRequested()
   {
      return enableAutomaticManipulationAbort.get();
   }

   public void reportManipulationAborted()
   {
      ManipulationAbortedStatus packet = new ManipulationAbortedStatus();
//      packet.setDestination(PacketDestination.UI);
      globalDataProducer.queueDataToSend(packet);
   }

   @Override
   public void receivedPacket(AutomaticManipulationAbortPacket packet)
   {
      if (packet == null)
         return;
      hasReceivedNewPacket.set(true);
      enableAutomaticManipulationAbort.set(packet.getEnable());
   }
}
