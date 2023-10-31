package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;

public class ROS2BehaviorTreePublisher
{
   private final BehaviorTreeStateMessage behaviorTreeMessage = new BehaviorTreeStateMessage();

   public void publish(BehaviorTreeState behaviorTreeState, ROS2PublishSubscribeAPI ros2PublishSubscribeAPI, ROS2IOTopicQualifier outgoingQualifier)
   {
      behaviorTreeState.toMessage(behaviorTreeMessage);
      ROS2BehaviorTreeMessageTools.clearLists(behaviorTreeMessage);

      packTreeToMessage(behaviorTreeState.getRootNode().getState());

      ros2PublishSubscribeAPI.publish(AutonomyAPI.BEAVIOR_TREE.getTopic(outgoingQualifier), behaviorTreeMessage);
   }

   private void packTreeToMessage(BehaviorTreeNodeState behaviorTreeNode)
   {
      ROS2BehaviorTreeMessageTools.packMessage(behaviorTreeNode, behaviorTreeMessage);

      for (Object child : behaviorTreeNode.getChildren())
      {
         packTreeToMessage((BehaviorTreeNodeState) child);
      }
   }
}
