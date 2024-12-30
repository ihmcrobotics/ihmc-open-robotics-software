package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTree;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeHighLayer;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.ros2.ROS2Topic;

public class ROS2BehaviorTreePublisher
{
   private final BehaviorTree behaviorTree;
   private final ROS2PublishSubscribeAPI ros2PublishSubscribeAPI;
   private final BehaviorTreeStateMessage behaviorTreeMessage = new BehaviorTreeStateMessage();
   private final ROS2Topic<BehaviorTreeStateMessage> topic;

   public ROS2BehaviorTreePublisher(BehaviorTree behaviorTree, ROS2PublishSubscribeAPI ros2PublishSubscribeAPI)
   {
      this.behaviorTree = behaviorTree;
      this.ros2PublishSubscribeAPI = ros2PublishSubscribeAPI;

      topic = AutonomyAPI.BEAVIOR_TREE.getTopic(behaviorTree.getCRDTInfo().getActorDesignation().getOutgoingQualifier());
   }

   public void publish()
   {
      behaviorTree.toMessage(behaviorTreeMessage);
      ROS2BehaviorTreeMessageTools.clearLists(behaviorTreeMessage);

      BehaviorTreeNodeHighLayer<?, ?, ?> rootNode = behaviorTree.getRootNode();
      if (rootNode != null)
      {
         packTreeToMessage(rootNode.getState());
      }

      ros2PublishSubscribeAPI.publish(topic, behaviorTreeMessage);
   }

   private void packTreeToMessage(BehaviorTreeNodeState behaviorTreeNode)
   {
      ROS2BehaviorTreeMessageTools.packMessage(behaviorTree.getCRDTInfo(), behaviorTreeNode, behaviorTreeMessage);

      for (Object child : behaviorTreeNode.getChildren())
      {
         packTreeToMessage((BehaviorTreeNodeState) child);
      }
   }
}
