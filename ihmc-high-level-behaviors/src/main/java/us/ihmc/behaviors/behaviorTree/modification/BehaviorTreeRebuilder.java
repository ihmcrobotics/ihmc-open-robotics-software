package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStateSupplier;

import javax.annotation.Nullable;

public interface BehaviorTreeRebuilder
{
   /**
    * @return replacement or null if the node was not previously in the tree
    */
   @Nullable
   BehaviorTreeNodeStateSupplier getReplacementNode(long id);

   BehaviorTreeStateModification getReplacementModification(long id, BehaviorTreeNodeStateSupplier parent);

   BehaviorTreeStateModification getClearSubtreeModification();

   BehaviorTreeStateModification getDestroyLeftoversModification();
}
