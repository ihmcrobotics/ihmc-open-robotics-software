package us.ihmc.valkyrie.hands.athena;

import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.NewMessageListener;
import valkyrie_msgs.msg.dds.AthenaTrajectoryMessage;

public class AthenaTrajectoryMessageSubscriber implements NewMessageListener<AthenaTrajectoryMessage>
{
   private final ConcurrentLinkedQueue<AthenaTrajectoryMessage> messageQueue = new ConcurrentLinkedQueue<>();
   private RobotSide robotSide;

   public AthenaTrajectoryMessageSubscriber(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   @Override
   public void onNewDataMessage(Subscriber<AthenaTrajectoryMessage> subscriber)
   {
      receivedPacket(subscriber.takeNextData());
   }

   public void receivedPacket(AthenaTrajectoryMessage ihmcMessage)
   {
      if (this.robotSide == null)
         messageQueue.add(ihmcMessage);
      else if (ihmcMessage.getRobotSide() == this.robotSide.toByte())
         messageQueue.add(ihmcMessage);
   }

   public AthenaTrajectoryMessage pollMessage()
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
