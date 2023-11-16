package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeLayer;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.ros2.ROS2Topic;

public class ROS2BehaviorTreePublisher
{
   private final BehaviorTreeState behaviorTreeState;
   private final ROS2PublishSubscribeAPI ros2PublishSubscribeAPI;
   private final BehaviorTreeStateMessage behaviorTreeMessage = new BehaviorTreeStateMessage();
   private final ROS2Topic<BehaviorTreeStateMessage> topic;

   public ROS2BehaviorTreePublisher(BehaviorTreeState behaviorTreeState, ROS2PublishSubscribeAPI ros2PublishSubscribeAPI)
   {
      this.behaviorTreeState = behaviorTreeState;
      this.ros2PublishSubscribeAPI = ros2PublishSubscribeAPI;

      topic = AutonomyAPI.BEAVIOR_TREE.getTopic(behaviorTreeState.getCRDTInfo().getActorDesignation().getOutgoingQualifier());
   }

   public void publish()
   {
      behaviorTreeState.toMessage(behaviorTreeMessage);
      ROS2BehaviorTreeMessageTools.clearLists(behaviorTreeMessage);

      BehaviorTreeNodeLayer<?, ?, ?, ?> rootNode = behaviorTreeState.getRootNode();
      if (rootNode != null)
      {
         packTreeToMessage(rootNode.getState());
      }

      ros2PublishSubscribeAPI.publish(topic, behaviorTreeMessage);
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
