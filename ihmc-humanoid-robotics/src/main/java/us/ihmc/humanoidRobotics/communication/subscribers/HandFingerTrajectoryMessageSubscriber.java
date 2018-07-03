package us.ihmc.humanoidRobotics.communication.subscribers;

import java.util.concurrent.ConcurrentLinkedQueue;

import controller_msgs.msg.dds.HandFingerTrajectoryMessage;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.NewMessageListener;

public class HandFingerTrajectoryMessageSubscriber implements NewMessageListener<HandFingerTrajectoryMessage>
{
   private final ConcurrentLinkedQueue<HandFingerTrajectoryMessage> messageQueue = new ConcurrentLinkedQueue<HandFingerTrajectoryMessage>();
   private RobotSide robotSide;

   public HandFingerTrajectoryMessageSubscriber(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   @Override
   public void onNewDataMessage(Subscriber<HandFingerTrajectoryMessage> subscriber)
   {
      receivedPacket(subscriber.takeNextData());
   }

   public void receivedPacket(HandFingerTrajectoryMessage ihmcMessage)
   {
      if (this.robotSide == null)
         messageQueue.add(ihmcMessage);
      else if (ihmcMessage.getRobotSide() == this.robotSide.toByte())
         messageQueue.add(ihmcMessage);
   }

   public HandFingerTrajectoryMessage pollMessage()
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