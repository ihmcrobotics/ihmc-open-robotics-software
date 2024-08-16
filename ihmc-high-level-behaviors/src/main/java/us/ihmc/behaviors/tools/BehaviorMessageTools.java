package us.ihmc.behaviors.tools;

import behavior_msgs.msg.dds.BehaviorTreeNodeMessage;
import behavior_msgs.msg.dds.BehaviorTreeMessage;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.behaviors.behaviorTree.*;

/**
 * @deprecated This class is kept only so that other code can stay compiling as
 *               we use it for reference in rewriting the behavior tree implementation.
 */
public class BehaviorMessageTools
{
   /**
    * Pack a behavior tree into a ROS 2 message. We can have recursive fields in our
    * messages, so we pack the tree in depth-first order.
    *
    * TODO: This is going to have to be fixed to pack different node types in
    *   appropriate fields in the ROS 2 message.
    */
   public static void packBehaviorTreeMessage(BehaviorTreeNodeExecutor<?, ?> treeNode, BehaviorTreeMessage behaviorTreeMessage)
   {
      BehaviorTreeNodeMessage nodeMessage = behaviorTreeMessage.getNodes().add();
//      if (treeNode.getState().getLastTickInstant()  != null)
//         MessageTools.toMessage(treeNode.getState().getLastTickInstant(), nodeMessage.getLastTickInstant());
      nodeMessage.setNodeName(treeNode.getDefinition().getName());
//      if (treeNode.getState().getStatus() != null)
//         nodeMessage.setPreviousStatus((byte) treeNode.getState().getStatus().ordinal());
//      else
         nodeMessage.setPreviousStatus((byte) -1);

      nodeMessage.setNumberOfChildren(treeNode.getChildren().size());
      // TODO: Pack status

      for (BehaviorTreeNodeExecutor<?, ?> child : treeNode.getChildren())
      {
         packBehaviorTreeMessage(child, behaviorTreeMessage);
      }
   }

   /**
    * We unpack a tree from a list of nodes using recursion and the number of
    * children of that node. We assume the ordering as packed in {@link #packBehaviorTreeMessage}.
    */
   public static BehaviorTreeNodeExecutor unpackBehaviorTreeMessage(BehaviorTreeMessage behaviorTreeMessage)
   {
      return unpackBehaviorTreeMessage(behaviorTreeMessage, new MutableInt());
   }

   private static BehaviorTreeNodeExecutor unpackBehaviorTreeMessage(BehaviorTreeMessage behaviorTreeMessage, MutableInt nodeIndex)
   {
      BehaviorTreeNodeMessage treeNodeMessage = behaviorTreeMessage.getNodes().get(nodeIndex.getAndIncrement());

      BehaviorTreeNodeExecutor behaviorTreeStatusNode = createBehaviorTreeNode(treeNodeMessage.getNodeTypeAsString());
      // The message will have 0s for a node that has not yet been ticked
//      if (treeNodeMessage.getLastTickInstant().getSecondsSinceEpoch() != 0)
//         behaviorTreeStatusNode.getState().setLastTickInstant(MessageTools.toInstant(treeNodeMessage.getLastTickInstant()));
      String name = treeNodeMessage.getNodeNameAsString();
      behaviorTreeStatusNode.getDefinition().setName(name);
      // Previous status will be -1 if the node has not been ticked yet
//      if (treeNodeMessage.getPreviousStatus() >= 0)
//         behaviorTreeStatusNode.getState().setStatus(BehaviorTreeNodeStatus.fromByte(treeNodeMessage.getPreviousStatus()));

      for (int i = 0; i < treeNodeMessage.getNumberOfChildren(); i++)
      {
         behaviorTreeStatusNode.getChildren().add(unpackBehaviorTreeMessage(behaviorTreeMessage, nodeIndex));
      }

      return behaviorTreeStatusNode;
   }

   private static BehaviorTreeNodeExecutor createBehaviorTreeNode(String typeName)
   {
      BehaviorTreeNodeExecutor behaviorTreeStatusNode = null;
      // TODO: We need to instantiate certain types of status nodes instead.
      //  They will need different functionality depending on their type.
//      if (typeName.equals(SequenceNode.class.getSimpleName()))
//         behaviorTreeStatusNode.setType(SequenceNode.class);
//      else if (typeName.equals(FallbackNode.class.getSimpleName()))
//         behaviorTreeStatusNode.setType(FallbackNode.class);
//      else if (typeName.equals(AsynchronousActionNode.class.getSimpleName()))
//         behaviorTreeStatusNode.setType(AsynchronousActionNode.class);
//      else if (typeName.equals(BehaviorTreeNodeState.class.getSimpleName()))
//         behaviorTreeStatusNode.setType(BehaviorTreeNodeState.class);
//      else if (typeName.equals(BehaviorTreeCondition.class.getSimpleName()))
//         behaviorTreeStatusNode.setType(BehaviorTreeCondition.class);
//      else if (typeName.equals(OneShotAction.class.getSimpleName()))
//         behaviorTreeStatusNode.setType(OneShotAction.class);
//      else if (typeName.equals(AlwaysSuccessfulAction.class.getSimpleName()))
//         behaviorTreeStatusNode.setType(AlwaysSuccessfulAction.class);
//      else
//         behaviorTreeStatusNode.setType(BehaviorTreeNodeState.class);
      return behaviorTreeStatusNode;
   }
}
