package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;

public class ChestTrajectoryMessageSubscriber implements PacketConsumer<ChestTrajectoryMessage>
{
   private final HumanoidGlobalDataProducer globalDataProducer;

   private final AtomicReference<ChestTrajectoryMessage> latestMessageReference = new AtomicReference<ChestTrajectoryMessage>(null);
   
   public ChestTrajectoryMessageSubscriber(HumanoidGlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;
   }

   public boolean isNewTrajectoryMessageAvailable()
   {
      return latestMessageReference.get() != null;
   }

   public ChestTrajectoryMessage pollMessage()
   {
      return latestMessageReference.getAndSet(null);
   }

   public void clearMessagesInQueue()
   {
      latestMessageReference.set(null);
   }

   @Override
   public void receivedPacket(ChestTrajectoryMessage chestTrajectoryMessage)
   {
      if (!PacketValidityChecker.validateChestTrajectoryMessage(chestTrajectoryMessage, globalDataProducer))
         return;

      latestMessageReference.set(chestTrajectoryMessage);
   }
}
