package us.ihmc.rdx.ui.behavior.tree.modification;

import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;

public class RDXBehaviorTreeDestroySubtreeBasics implements RDXBehaviorTreeModification
{
   private final RDXBehaviorTreeNode subtreeToRebuild;

   public RDXBehaviorTreeDestroySubtreeBasics(RDXBehaviorTreeNode subtreeToRebuild)
   {
      this.subtreeToRebuild = subtreeToRebuild;
   }

   @Override
   public void performOperation()
   {
      destroyChildren(subtreeToRebuild);
   }

   private void destroyChildren(RDXBehaviorTreeNode localNode)
   {
      for (RDXBehaviorTreeNode child : localNode.getChildren())
      {
         destroyChildren(child);
      }

      localNode.getChildren().clear();
      localNode.destroy();
   }
}

