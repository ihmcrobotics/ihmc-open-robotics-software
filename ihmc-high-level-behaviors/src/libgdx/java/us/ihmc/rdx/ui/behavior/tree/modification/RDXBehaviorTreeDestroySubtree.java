package us.ihmc.rdx.ui.behavior.tree.modification;

import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeStateDestroySubtree;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;

public class RDXBehaviorTreeDestroySubtree extends RDXBehaviorTreeDestroySubtreeBasics
{
   private final BehaviorTreeStateDestroySubtree stateDestroySubtree;

   public RDXBehaviorTreeDestroySubtree(RDXBehaviorTreeNode subtreeToRebuild)
   {
      super(subtreeToRebuild);

      stateDestroySubtree = new BehaviorTreeStateDestroySubtree(subtreeToRebuild.getState());
   }

   @Override
   public void performOperation()
   {
      super.performOperation();

      stateDestroySubtree.performOperation();
   }
}

