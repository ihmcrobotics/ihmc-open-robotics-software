package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;

public class PelvisTrajectoryMessageSubscriber implements PacketConsumer<PelvisTrajectoryMessage>
{
   private final HumanoidGlobalDataProducer globalDataProducer;

   private final AtomicReference<PelvisTrajectoryMessage> latestMessageReference = new AtomicReference<PelvisTrajectoryMessage>(null);

   public PelvisTrajectoryMessageSubscriber(HumanoidGlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;

      globalDataProducer.attachListener(PelvisTrajectoryMessage.class, this);
   }

   public boolean isNewTrajectoryMessageAvailable()
   {
      return latestMessageReference.get() != null;
   }

   public PelvisTrajectoryMessage pollMessage()
   {
      return latestMessageReference.getAndSet(null);
   }

   public void clearMessagesInQueue()
   {
      latestMessageReference.set(null);
   }

   @Override
   public void receivedPacket(PelvisTrajectoryMessage pelvisTrajectoryMessage)
   {
      String errorMessage = PacketValidityChecker.validatePelvisTrajectoryMessage(pelvisTrajectoryMessage);
      if (errorMessage != null)
      {
         if (globalDataProducer != null)
            globalDataProducer.notifyInvalidPacketReceived(pelvisTrajectoryMessage.getClass(), errorMessage);
         return;
      }

      latestMessageReference.set(pelvisTrajectoryMessage);
   }
}
