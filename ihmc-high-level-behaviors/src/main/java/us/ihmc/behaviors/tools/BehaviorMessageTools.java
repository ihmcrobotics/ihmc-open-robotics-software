package us.ihmc.behaviors.tools;

import behavior_msgs.msg.dds.BehaviorTreeNodeMessage;
import behavior_msgs.msg.dds.BehaviorTreeMessage;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.behaviors.tools.behaviorTree.*;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.log.LogTools;

import java.util.Stack;

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
      nodeMessage.setNodeType(treeNode.getType().getSimpleName());
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
      return unpackBehaviorTreeMessage(behaviorTreeMessage, new MutableInt(), new Stack<>());
   }

   private static BehaviorTreeNodeBasics unpackBehaviorTreeMessage(BehaviorTreeMessage behaviorTreeMessage, MutableInt nodeIndex, Stack<Integer> childrenStack)
   {
      BehaviorTreeNodeMessage treeNodeMessage = behaviorTreeMessage.getNodes().get(nodeIndex.getAndIncrement());

      BehaviorTreeNode behaviorTreeNode = createBehaviorTreeNode(treeNodeMessage.getNodeTypeAsString());
      behaviorTreeNode.setLastTickInstant(MessageTools.toInstant(treeNodeMessage.getLastTickInstant()));

      // TODO ...

      return behaviorTreeNode;
   }

   /**
    * FIXME: The design of this stuff seems terrible, we need to review it.
    */
   private static BehaviorTreeNode createBehaviorTreeNode(String typeName)
   {
      BehaviorTreeNode behaviorTreeNode;
      if (typeName.equals(SequenceNode.class.getSimpleName()))
      {
         behaviorTreeNode = new SequenceNode();
      }
      else if (typeName.equals(FallbackNode.class.getSimpleName()))
      {
         behaviorTreeNode = new FallbackNode();
      }
      else if (typeName.equals(AsynchronousActionNode.class.getSimpleName()))
      {
         behaviorTreeNode = new AsynchronousActionNode()
         {
            @Override
            public void resetInternal()
            {

            }
         };
      }
      else if (typeName.equals(BehaviorTreeAction.class.getSimpleName()))
      {
         behaviorTreeNode = new BehaviorTreeAction()
         {
            @Override
            public BehaviorTreeNodeStatus tickInternal()
            {
               return null;
            }
         };
      }
      else if (typeName.equals(BehaviorTreeCondition.class.getSimpleName()))
      {
         behaviorTreeNode = new BehaviorTreeCondition(() -> true);
      }
      else if (typeName.equals(OneShotAction.class.getSimpleName()))
      {
         behaviorTreeNode = new OneShotAction(() -> { });
      }
      else if (typeName.equals(AlwaysSuccessfulAction.class.getSimpleName()))
      {
         behaviorTreeNode = new AlwaysSuccessfulAction(() -> { });
      }
      else
      {
         LogTools.warn("Not sure what to create here...");
         behaviorTreeNode = new BehaviorTreeNode()
         {
            @Override
            public BehaviorTreeNodeStatus tickInternal()
            {
               return BehaviorTreeNodeStatus.SUCCESS;
            }
         };
      }
      return behaviorTreeNode;
   }
}
