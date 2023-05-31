package us.ihmc.valkyrie.hands;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.avatar.handControl.HandFingerTrajectoryMessagePublisher;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.valkyrie.hands.athena.AthenaHandModel.AthenaFingerMotorName;
import us.ihmc.valkyrie.network.ValkyrieMessageTools;
import valkyrie_msgs.msg.dds.AthenaTrajectoryMessage;

public class ValkyrieFingerTrajectoryMessagePublisher implements HandFingerTrajectoryMessagePublisher
{
   private final IHMCROS2Publisher<AthenaTrajectoryMessage> publisher;

   public ValkyrieFingerTrajectoryMessagePublisher(ROS2Node ros2Node, ROS2Topic inputTopic)
   {
      publisher = ROS2Tools.createPublisherTypeNamed(ros2Node, AthenaTrajectoryMessage.class, inputTopic);
   }

   @Override
   public void sendFingerTrajectoryMessage(RobotSide robotSide, TDoubleArrayList desiredPositions, TDoubleArrayList trajectoryTimes)
   {
      if (desiredPositions.size() != trajectoryTimes.size())
         throw new RuntimeException("Inconsistent array lengths.");

      AthenaTrajectoryMessage message = new AthenaTrajectoryMessage();
      message.setRobotSide(robotSide.toByte());

      for (int i = 0; i < desiredPositions.size(); i++)
      {
         ValkyrieMessageTools.appendDesiredFingerConfiguration(AthenaFingerMotorName.values[i], trajectoryTimes.get(i), desiredPositions.get(i), message);
      }

      publisher.publish(message);
   }
}
