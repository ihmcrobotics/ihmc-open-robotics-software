package us.ihmc.behaviors.tools;

import behavior_msgs.msg.dds.BehaviorTreeNodeMessage;
import behavior_msgs.msg.dds.BehaviorTreeMessage;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.behaviors.tools.behaviorTree.*;
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
      MessageTools.toMessage(treeNode.getLastTickInstant(), nodeMessage.getLastTickInstant());
      nodeMessage.setNodeName(treeNode.getName());
      nodeMessage.setNodeType(treeNode.getType());
      nodeMessage.setPreviousStatus((byte) treeNode.getPreviousStatus().ordinal());

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
    * We unpack a tree from a list of nodes using a stack that hold the number of
    * children of that node. We assume the ordering as packed in {@link #packBehaviorTreeMessage}.
    */
   public static BehaviorTreeNodeBasics unpackBehaviorTreeMessage(BehaviorTreeMessage behaviorTreeMessage)
   {
      return unpackBehaviorTreeMessage(behaviorTreeMessage, new MutableInt());
   }

   private static BehaviorTreeNodeBasics unpackBehaviorTreeMessage(BehaviorTreeMessage behaviorTreeMessage, MutableInt nodeIndex)
   {
      BehaviorTreeNodeMessage treeNodeMessage = behaviorTreeMessage.getNodes().get(nodeIndex.getAndIncrement());

      BehaviorTreeStatusNode behaviorTreeStatusNode = new BehaviorTreeStatusNode();
      behaviorTreeStatusNode.setLastTickInstant(MessageTools.toInstant(treeNodeMessage.getLastTickInstant()));
      behaviorTreeStatusNode.setName(treeNodeMessage.getNodeNameAsString());
      behaviorTreeStatusNode.setType(treeNodeMessage.getNodeTypeAsString());
      behaviorTreeStatusNode.setPreviousStatus(BehaviorTreeNodeStatus.fromByte(treeNodeMessage.getPreviousStatus()));
      behaviorTreeStatusNode.setHasBeenClocked(treeNodeMessage.getHasBeenClocked());

      for (int i = 0; i < treeNodeMessage.getNumberOfChildren(); i++)
      {
         behaviorTreeStatusNode.getChildren().add(unpackBehaviorTreeMessage(behaviorTreeMessage, nodeIndex));
      }

      return behaviorTreeStatusNode;
   }
}
