package us.ihmc.behaviors.behaviorTree.control;

import behavior_msgs.msg.dds.GotoNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.LeafNodeState;
import us.ihmc.log.LogTools;

import java.util.List;

public class GotoNodeState extends LeafNodeState<GotoNodeDefinition>
{
   public GotoNodeState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new GotoNodeDefinition(rootNode.getDefinition()), rootNode);
   }

   @Override
   public void validateDefinition(List<LeafNodeState<?>> leaves)
   {
      super.validateDefinition(leaves);

      if (definition.getNodeToGotoIsInvalid())
      {
         // We need to find the node by name
         // This happens when we load from JSON
         for (BehaviorTreeNodeState<?> leaf : leaves)
         {
            if (leaf.getDefinition().getName().equals(definition.getNodeToGotoName()))
            {
               definition.setNodeToGoto(leaf.getID(), definition.getNodeToGotoName());
               break;
            }
         }
      }
      else if (definition.getNodeToGotoID() >= 0)
      {
         // Dynamically update the node name -- it can change independently of the node's ID
         // This is necessary for saving the definition
         for (BehaviorTreeNodeState<?> leaf : leaves)
         {
            if (leaf.getID() == definition.getNodeToGotoID())
            {
               definition.setNodeToGotoName(leaf.getDefinition().getName());
            }
         }
      }
   }

   @Override
   public void update()
   {
      super.update();
   }

   public void toMessage(GotoNodeStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(GotoNodeStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }

   public LeafNodeState<?> findNodeToGoto()
   {
      BehaviorTreeNodeState<?> nodeToGoto = rootNode.getIDToNodeMap().get(definition.getNodeToGotoID());
      if (nodeToGoto instanceof LeafNodeState<?> leafState)
      {
         return leafState;
      }
      else
      {
         LogTools.error("Node to goto is not a leaf node.");
         return null;
      }
   }
}
