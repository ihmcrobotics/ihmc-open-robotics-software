package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandDesiredConfigurationSubscriber implements PacketConsumer<HandDesiredConfigurationMessage>
{
   private final ConcurrentLinkedQueue<HandDesiredConfigurationMessage> messageQueue = new ConcurrentLinkedQueue<HandDesiredConfigurationMessage>();
   private RobotSide robotSide;

   public HandDesiredConfigurationSubscriber(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void receivedPacket(HandDesiredConfigurationMessage ihmcMessage)
   {
      if (this.robotSide == null)
         messageQueue.add(ihmcMessage);
      else if (ihmcMessage.getRobotSide() == this.robotSide)
         messageQueue.add(ihmcMessage);
   }

   public HandDesiredConfigurationMessage pollMessage()
   {
      return messageQueue.poll();
   }

   public boolean isNewDesiredConfigurationAvailable()
   {
      return !messageQueue.isEmpty();
   }

   public RobotSide getSide()
   {
      return robotSide;
   }
}
