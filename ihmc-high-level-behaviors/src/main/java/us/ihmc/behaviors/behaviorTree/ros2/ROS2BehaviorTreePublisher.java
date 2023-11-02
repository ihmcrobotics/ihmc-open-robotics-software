package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.log.LogTools;

public class ROS2BehaviorTreePublisher
{
   private final BehaviorTreeStateMessage behaviorTreeMessage = new BehaviorTreeStateMessage();

   int count = 0;

   public void publish(BehaviorTreeState behaviorTreeState, ROS2PublishSubscribeAPI ros2PublishSubscribeAPI, ROS2ActorDesignation actorDesignation)
   {
      behaviorTreeState.toMessage(behaviorTreeMessage);
      ROS2BehaviorTreeMessageTools.clearLists(behaviorTreeMessage);

      if (actorDesignation == ROS2ActorDesignation.ROBOT)
      {
         count++;
      }
      else
      {
         count++;
      }

      BehaviorTreeNodeExtension<?, ?, ?, ?> rootNode = behaviorTreeState.getRootNode();
      if (rootNode == null)
      {
         LogTools.info("Root node null");
      }
      else
      {
         packTreeToMessage(rootNode.getState());
      }

      ros2PublishSubscribeAPI.publish(AutonomyAPI.BEAVIOR_TREE.getTopic(actorDesignation.getOutgoingQualifier()), behaviorTreeMessage);
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
