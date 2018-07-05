package us.ihmc.humanoidRobotics.communication.subscribers;

import java.util.concurrent.ConcurrentLinkedQueue;

import controller_msgs.msg.dds.ValkyrieHandFingerTrajectoryMessage;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.NewMessageListener;

public class ValkyrieHandFingerTrajectoryMessageSubscriber implements NewMessageListener<ValkyrieHandFingerTrajectoryMessage>
{
   private final ConcurrentLinkedQueue<ValkyrieHandFingerTrajectoryMessage> messageQueue = new ConcurrentLinkedQueue<ValkyrieHandFingerTrajectoryMessage>();
   private RobotSide robotSide;

   public ValkyrieHandFingerTrajectoryMessageSubscriber(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   @Override
   public void onNewDataMessage(Subscriber<ValkyrieHandFingerTrajectoryMessage> subscriber)
   {
      receivedPacket(subscriber.takeNextData());
   }

   public void receivedPacket(ValkyrieHandFingerTrajectoryMessage ihmcMessage)
   {
      if (this.robotSide == null)
         messageQueue.add(ihmcMessage);
      else if (ihmcMessage.getRobotSide() == this.robotSide.toByte())
         messageQueue.add(ihmcMessage);
   }

   public ValkyrieHandFingerTrajectoryMessage pollMessage()
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
