package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ArmDesiredAccelerationsMessageSubscriber implements PacketConsumer<ArmDesiredAccelerationsMessage>
{
   private final HumanoidGlobalDataProducer globalDataProducer;

   private final SideDependentList<AtomicReference<ArmDesiredAccelerationsMessage>> latestMessageReferences = new SideDependentList<>();
   
   public ArmDesiredAccelerationsMessageSubscriber(HumanoidGlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;

      for (RobotSide robotSide : RobotSide.values)
         latestMessageReferences.put(robotSide, new AtomicReference<ArmDesiredAccelerationsMessage>(null));

      globalDataProducer.attachListener(ArmDesiredAccelerationsMessage.class, this);
   }

   public boolean isNewControlMessageAvailable(RobotSide robotSide)
   {
      return latestMessageReferences.get(robotSide).get() != null;
   }

   public ArmDesiredAccelerationsMessage pollMessage(RobotSide robotSide)
   {
      return latestMessageReferences.get(robotSide).getAndSet(null);
   }

   public void clearMessagesInQueue()
   {
      for (RobotSide robotSide : RobotSide.values)
         latestMessageReferences.get(robotSide).set(null);
   }

   @Override
   public void receivedPacket(ArmDesiredAccelerationsMessage armDesiredAccelerationsMessage)
   {
      if (!PacketValidityChecker.validateArmDesiredAccelerationsMessage(armDesiredAccelerationsMessage, globalDataProducer))
         return;

      RobotSide robotSide = armDesiredAccelerationsMessage.getRobotSide();
      latestMessageReferences.get(robotSide).set(armDesiredAccelerationsMessage);
   }
}
