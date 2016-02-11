package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;

public class EndEffectorLoadBearingMessageSubscriber implements PacketConsumer<EndEffectorLoadBearingMessage>
{
   private final HumanoidGlobalDataProducer globalDataProducer;

   private final AtomicReference<EndEffectorLoadBearingMessage> latestMessageReference = new AtomicReference<EndEffectorLoadBearingMessage>(null);
   
   public EndEffectorLoadBearingMessageSubscriber(HumanoidGlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;
   }

   public boolean isNewMessageAvailable()
   {
      return latestMessageReference.get() != null;
   }

   public EndEffectorLoadBearingMessage pollMessage()
   {
      return latestMessageReference.getAndSet(null);
   }

   public void clearMessagesInQueue()
   {
      latestMessageReference.set(null);
   }

   @Override
   public void receivedPacket(EndEffectorLoadBearingMessage endEffectorLoadBearingMessage)
   {
      if (!PacketValidityChecker.validateEndEffectorLoadBearingMessage(endEffectorLoadBearingMessage, globalDataProducer))
         return;

      latestMessageReference.set(endEffectorLoadBearingMessage);
   }
}
