package us.ihmc.humanoidRobotics.communication.subscribers;

import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandDesiredConfigurationMessageSubscriber implements PacketConsumer<HandDesiredConfigurationMessage>
{
   private final ConcurrentLinkedQueue<HandDesiredConfigurationMessage> messageQueue = new ConcurrentLinkedQueue<HandDesiredConfigurationMessage>();
   private RobotSide robotSide;

   public HandDesiredConfigurationMessageSubscriber(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public void receivedPacket(HandDesiredConfigurationMessage ihmcMessage)
   {
      if (this.robotSide == null)
         messageQueue.add(ihmcMessage);
      else if (ihmcMessage.getRobotSide() == this.robotSide.toByte())
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
