package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;

public class PelvisHeightTrajectoryMessageSubscriber implements PacketConsumer<PelvisHeightTrajectoryMessage>
{
   private final HumanoidGlobalDataProducer globalDataProducer;

   private final AtomicReference<PelvisHeightTrajectoryMessage> latestMessageReference = new AtomicReference<PelvisHeightTrajectoryMessage>(null);

   public PelvisHeightTrajectoryMessageSubscriber(HumanoidGlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;

      globalDataProducer.attachListener(PelvisHeightTrajectoryMessage.class, this);
   }

   public boolean isNewTrajectoryMessageAvailable()
   {
      return latestMessageReference.get() != null;
   }

   public PelvisHeightTrajectoryMessage pollMessage()
   {
      return latestMessageReference.getAndSet(null);
   }

   public void clearMessagesInQueue()
   {
      latestMessageReference.set(null);
   }

   @Override
   public void receivedPacket(PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage)
   {
      if (!PacketValidityChecker.validatePelvisHeightTrajectoryMessage(pelvisHeightTrajectoryMessage, globalDataProducer))
         return;

      latestMessageReference.set(pelvisHeightTrajectoryMessage);
   }
}
