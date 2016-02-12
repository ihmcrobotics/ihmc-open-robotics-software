package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ArmTrajectoryMessageSubscriber implements PacketConsumer<ArmTrajectoryMessage>
{
   private final HumanoidGlobalDataProducer globalDataProducer;

   private final SideDependentList<AtomicReference<ArmTrajectoryMessage>> latestMessageReferences = new SideDependentList<>(new AtomicReference<ArmTrajectoryMessage>(null), new AtomicReference<ArmTrajectoryMessage>(null));
   
   public ArmTrajectoryMessageSubscriber(HumanoidGlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;
   }

   public boolean isNewTrajectoryMessageAvailable(RobotSide robotSide)
   {
      return latestMessageReferences.get(robotSide).get() != null;
   }

   public ArmTrajectoryMessage pollMessage(RobotSide robotSide)
   {
      return latestMessageReferences.get(robotSide).getAndSet(null);
   }

   public void clearMessagesInQueue()
   {
      for (RobotSide robotSide : RobotSide.values)
         latestMessageReferences.get(robotSide).set(null);
   }

   @Override
   public void receivedPacket(ArmTrajectoryMessage armTrajectoryMessage)
   {
      if (!PacketValidityChecker.validateArmTrajectoryMessage(armTrajectoryMessage, globalDataProducer))
         return;

      RobotSide robotSide = armTrajectoryMessage.getRobotSide();
      latestMessageReferences.get(robotSide).set(armTrajectoryMessage);
   }
}
