package us.ihmc.humanoidRobotics.communication.subscribers;

import java.util.concurrent.ConcurrentLinkedQueue;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.NewMessageListener;

public class HandDesiredConfigurationMessageSubscriber implements NewMessageListener<HandDesiredConfigurationMessage>
{
   private final ConcurrentLinkedQueue<HandDesiredConfigurationMessage> messageQueue = new ConcurrentLinkedQueue<HandDesiredConfigurationMessage>();
   private RobotSide robotSide;

   public HandDesiredConfigurationMessageSubscriber(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   @Override
   public void onNewDataMessage(Subscriber<HandDesiredConfigurationMessage> subscriber)
   {
      receivedPacket(subscriber.takeNextData());
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
