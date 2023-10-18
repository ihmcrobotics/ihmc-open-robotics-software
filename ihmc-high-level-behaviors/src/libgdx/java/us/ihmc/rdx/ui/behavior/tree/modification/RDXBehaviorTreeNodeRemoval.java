package us.ihmc.rdx.ui.behavior.tree.modification;

import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeNodeStateRemoval;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeStateModification;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;

public class RDXBehaviorTreeNodeRemoval implements BehaviorTreeStateModification
{
   private final BehaviorTreeNodeStateRemoval stateRemoval;

   private final RDXBehaviorTreeNode nodeToRemove;
   private final RDXBehaviorTreeNode rootNode;

   public RDXBehaviorTreeNodeRemoval(RDXBehaviorTreeNode nodeToRemove, RDXBehaviorTreeNode rootNode)
   {
      this.nodeToRemove = nodeToRemove;
      this.rootNode = rootNode;

      stateRemoval = new BehaviorTreeNodeStateRemoval(nodeToRemove.getState(), rootNode.getState());
   }

   @Override
   public void performOperation()
   {
      findAndRemove(rootNode);

      stateRemoval.performOperation();
   }

   private void findAndRemove(RDXBehaviorTreeNode node)
   {
      if (!node.getChildren().remove(nodeToRemove))
      {
         for (RDXBehaviorTreeNode child : node.getChildren())
         {
            findAndRemove(child);
         }
      }
   }
}
