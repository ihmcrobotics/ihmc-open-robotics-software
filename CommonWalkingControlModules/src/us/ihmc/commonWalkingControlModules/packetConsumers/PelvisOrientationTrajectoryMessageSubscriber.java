package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;

public class PelvisOrientationTrajectoryMessageSubscriber implements PacketConsumer<PelvisOrientationTrajectoryMessage>
{
   private final HumanoidGlobalDataProducer globalDataProducer;

   private final AtomicReference<PelvisOrientationTrajectoryMessage> latestMessageReference = new AtomicReference<PelvisOrientationTrajectoryMessage>(null);

   public PelvisOrientationTrajectoryMessageSubscriber(HumanoidGlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;

      globalDataProducer.attachListener(PelvisOrientationTrajectoryMessage.class, this);
   }

   public boolean isNewTrajectoryMessageAvailable()
   {
      return latestMessageReference.get() != null;
   }

   public PelvisOrientationTrajectoryMessage pollMessage()
   {
      return latestMessageReference.getAndSet(null);
   }

   public void clearMessagesInQueue()
   {
      latestMessageReference.set(null);
   }

   @Override
   public void receivedPacket(PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage)
   {
      String errorMessage = PacketValidityChecker.validatePelvisOrientationTrajectoryMessage(pelvisOrientationTrajectoryMessage);
      if (errorMessage != null)
      {
         if (globalDataProducer != null)
            globalDataProducer.notifyInvalidPacketReceived(pelvisOrientationTrajectoryMessage.getClass(), errorMessage);
         return;
      }

      latestMessageReference.set(pelvisOrientationTrajectoryMessage);
   }
}
