package us.ihmc.behaviors.tools;

import behavior_msgs.msg.dds.BehaviorTreeNodeMessage;
import behavior_msgs.msg.dds.BehaviorTreeMessage;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.behaviors.behaviorTree.*;
import us.ihmc.communication.packets.MessageTools;

public class BehaviorMessageTools
{
   /**
    * Pack a behavior tree into a ROS 2 message. We can have recursive fields in our
    * messages, so we pack the tree in depth-first order.
    */
   public static void packBehaviorTreeMessage(BehaviorTreeNodeBasics treeNode, BehaviorTreeMessage behaviorTreeMessage)
   {
      BehaviorTreeNodeMessage nodeMessage = behaviorTreeMessage.getNodes().add();
      if (treeNode.getLastTickInstant()  != null)
         MessageTools.toMessage(treeNode.getLastTickInstant(), nodeMessage.getLastTickInstant());
      nodeMessage.setNodeName(treeNode.getName());
      nodeMessage.setNodeType(treeNode.getType().getSimpleName());
      if (treeNode.getPreviousStatus() != null)
         nodeMessage.setPreviousStatus((byte) treeNode.getPreviousStatus().ordinal());
      else
         nodeMessage.setPreviousStatus((byte) -1);

      if (treeNode instanceof BehaviorTreeControlFlowNodeBasics controlFlowTreeNode)
      {
         nodeMessage.setNumberOfChildren(controlFlowTreeNode.getChildren().size());
         nodeMessage.setHasBeenClocked(controlFlowTreeNode.getHasBeenClocked());

         for (BehaviorTreeNodeBasics child : controlFlowTreeNode.getChildren())
         {
            packBehaviorTreeMessage(child, behaviorTreeMessage);
         }
      }
      else
      {
         nodeMessage.setNumberOfChildren(0);
      }
   }

   /**
    * We unpack a tree from a list of nodes using recursion and the number of
    * children of that node. We assume the ordering as packed in {@link #packBehaviorTreeMessage}.
    */
   public static BehaviorTreeNodeBasics unpackBehaviorTreeMessage(BehaviorTreeMessage behaviorTreeMessage)
   {
      return unpackBehaviorTreeMessage(behaviorTreeMessage, new MutableInt());
   }

   private static BehaviorTreeNodeBasics unpackBehaviorTreeMessage(BehaviorTreeMessage behaviorTreeMessage, MutableInt nodeIndex)
   {
      BehaviorTreeNodeMessage treeNodeMessage = behaviorTreeMessage.getNodes().get(nodeIndex.getAndIncrement());

      BehaviorTreeStatusNode behaviorTreeStatusNode = createBehaviorTreeNode(treeNodeMessage.getNodeTypeAsString());
      // The message will have 0s for a node that has not yet been ticked
      if (treeNodeMessage.getLastTickInstant().getSecondsSinceEpoch() != 0)
         behaviorTreeStatusNode.setLastTickInstant(MessageTools.toInstant(treeNodeMessage.getLastTickInstant()));
      behaviorTreeStatusNode.setName(treeNodeMessage.getNodeNameAsString());
      // Previous status will be -1 if the node has not been ticked yet
      if (treeNodeMessage.getPreviousStatus() >= 0)
         behaviorTreeStatusNode.setPreviousStatus(BehaviorTreeNodeStatus.fromByte(treeNodeMessage.getPreviousStatus()));
      behaviorTreeStatusNode.setHasBeenClocked(treeNodeMessage.getHasBeenClocked());

      for (int i = 0; i < treeNodeMessage.getNumberOfChildren(); i++)
      {
         behaviorTreeStatusNode.getChildren().add(unpackBehaviorTreeMessage(behaviorTreeMessage, nodeIndex));
      }

      return behaviorTreeStatusNode;
   }

   private static BehaviorTreeStatusNode createBehaviorTreeNode(String typeName)
   {
      BehaviorTreeStatusNode behaviorTreeStatusNode = new BehaviorTreeStatusNode();
      if (typeName.equals(SequenceNode.class.getSimpleName()))
         behaviorTreeStatusNode.setType(SequenceNode.class);
      else if (typeName.equals(FallbackNode.class.getSimpleName()))
         behaviorTreeStatusNode.setType(FallbackNode.class);
      else if (typeName.equals(AsynchronousActionNode.class.getSimpleName()))
         behaviorTreeStatusNode.setType(AsynchronousActionNode.class);
      else if (typeName.equals(BehaviorTreeAction.class.getSimpleName()))
         behaviorTreeStatusNode.setType(BehaviorTreeAction.class);
      else if (typeName.equals(BehaviorTreeCondition.class.getSimpleName()))
         behaviorTreeStatusNode.setType(BehaviorTreeCondition.class);
      else if (typeName.equals(OneShotAction.class.getSimpleName()))
         behaviorTreeStatusNode.setType(OneShotAction.class);
      else if (typeName.equals(AlwaysSuccessfulAction.class.getSimpleName()))
         behaviorTreeStatusNode.setType(AlwaysSuccessfulAction.class);
      else
         behaviorTreeStatusNode.setType(BehaviorTreeNode.class);
      return behaviorTreeStatusNode;
   }
}
