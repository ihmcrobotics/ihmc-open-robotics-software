package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class HandTrajectoryMessageSubscriber implements PacketConsumer<HandTrajectoryMessage>
{
   private final HumanoidGlobalDataProducer globalDataProducer;

   private final SideDependentList<AtomicReference<HandTrajectoryMessage>> latestMessageReferences = new SideDependentList<>(new AtomicReference<HandTrajectoryMessage>(null), new AtomicReference<HandTrajectoryMessage>(null));
   
   public HandTrajectoryMessageSubscriber(HumanoidGlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;
   }

   public boolean isNewTrajectoryMessageAvailable(RobotSide robotSide)
   {
      return latestMessageReferences.get(robotSide).get() != null;
   }

   public HandTrajectoryMessage pollMessage(RobotSide robotSide)
   {
      return latestMessageReferences.get(robotSide).getAndSet(null);
   }

   public void clearMessagesInQueue()
   {
      for (RobotSide robotSide : RobotSide.values)
         latestMessageReferences.get(robotSide).set(null);
   }

   @Override
   public void receivedPacket(HandTrajectoryMessage handTrajectoryMessage)
   {
      if (!PacketValidityChecker.validateHandTrajectoryMessage(handTrajectoryMessage, globalDataProducer))
         return;

      RobotSide robotSide = handTrajectoryMessage.getRobotSide();
      latestMessageReferences.get(robotSide).set(handTrajectoryMessage);
   }
}
