package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.robotics.robotSide.RobotSide;

public class WholeBodyTrajectoryMessageSubscriber implements PacketConsumer<WholeBodyTrajectoryMessage>
{
   private final HandTrajectoryMessageSubscriber handTrajectoryMessageSubscriber;
   private final ArmTrajectoryMessageSubscriber armTrajectoryMessageSubscriber;
   private final ChestTrajectoryMessageSubscriber chestTrajectoryMessageSubscriber;
   private final PelvisTrajectoryMessageSubscriber pelvisTrajectoryMessageSubscriber;
   private final FootTrajectoryMessageSubscriber footTrajectoryMessageSubscriber;

   private final HumanoidGlobalDataProducer globalDataProducer;

   public WholeBodyTrajectoryMessageSubscriber(HandTrajectoryMessageSubscriber handTrajectoryMessageSubscriber,
         ArmTrajectoryMessageSubscriber armTrajectoryMessageSubscriber, ChestTrajectoryMessageSubscriber chestTrajectoryMessageSubscriber,
         PelvisTrajectoryMessageSubscriber pelvisTrajectoryMessageSubscriber, FootTrajectoryMessageSubscriber footTrajectoryMessageSubscriber,
         HumanoidGlobalDataProducer globalDataProducer)
   {
      this.handTrajectoryMessageSubscriber = handTrajectoryMessageSubscriber;
      this.armTrajectoryMessageSubscriber = armTrajectoryMessageSubscriber;
      this.chestTrajectoryMessageSubscriber = chestTrajectoryMessageSubscriber;
      this.pelvisTrajectoryMessageSubscriber = pelvisTrajectoryMessageSubscriber;
      this.footTrajectoryMessageSubscriber = footTrajectoryMessageSubscriber;

      this.globalDataProducer = globalDataProducer;

      globalDataProducer.attachListener(WholeBodyTrajectoryMessage.class, this);
   }

   @Override
   public void receivedPacket(WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage)
   {
      if (PacketValidityChecker.validatePacket(wholeBodyTrajectoryMessage) != null)
         return;
      if (!wholeBodyTrajectoryMessage.checkRobotSideConsistency())
      {
         globalDataProducer.notifyInvalidPacketReceived(wholeBodyTrajectoryMessage.getClass(), "The robotSide of a field is inconsistent with its name.");
         return;
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         handTrajectoryMessageSubscriber.receivedPacket(wholeBodyTrajectoryMessage.getHandTrajectoryMessage(robotSide));
         armTrajectoryMessageSubscriber.receivedPacket(wholeBodyTrajectoryMessage.getArmTrajectoryMessage(robotSide));
      }

      chestTrajectoryMessageSubscriber.receivedPacket(wholeBodyTrajectoryMessage.getChestTrajectoryMessage());
      pelvisTrajectoryMessageSubscriber.receivedPacket(wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage());

      for (RobotSide robotSide : RobotSide.values)
         footTrajectoryMessageSubscriber.receivedPacket(wholeBodyTrajectoryMessage.getFootTrajectoryMessage(robotSide));
   }
}
