package us.ihmc.rdx.ui.behavior.tree.modification;

import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeStateNodeRemoval;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;

public class RDXBehaviorTreeNodeRemoval implements RDXBehaviorTreeModification
{
   private final RDXBehaviorTreeNode nodeToRemove;
   private final RDXBehaviorTreeNode rootNode;

   private final BehaviorTreeStateNodeRemoval stateRemoval;

   public RDXBehaviorTreeNodeRemoval(RDXBehaviorTreeNode nodeToRemove, RDXBehaviorTreeNode rootNode)
   {
      this.nodeToRemove = nodeToRemove;
      this.rootNode = rootNode;

      stateRemoval = new BehaviorTreeStateNodeRemoval(nodeToRemove.getState(), rootNode.getState());
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
