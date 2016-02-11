package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;

public class HeadTrajectoryMessageSubscriber implements PacketConsumer<HeadTrajectoryMessage>
{
   private final HumanoidGlobalDataProducer globalDataProducer;

   private final AtomicReference<HeadTrajectoryMessage> latestMessageReference = new AtomicReference<HeadTrajectoryMessage>(null);
   
   public HeadTrajectoryMessageSubscriber(HumanoidGlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;
   }

   public boolean isNewTrajectoryMessageAvailable()
   {
      return latestMessageReference.get() != null;
   }

   public HeadTrajectoryMessage pollMessage()
   {
      return latestMessageReference.getAndSet(null);
   }

   public void clearMessagesInQueue()
   {
      latestMessageReference.set(null);
   }

   @Override
   public void receivedPacket(HeadTrajectoryMessage headTrajectoryMessage)
   {
      if (!PacketValidityChecker.validateHeadTrajectoryMessage(headTrajectoryMessage, globalDataProducer))
         return;

      latestMessageReference.set(headTrajectoryMessage);
   }
}
