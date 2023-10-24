package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.communication.crdt.Freezable;

/**
 * Adds a node to a parent, freezing the parent if it's freezable.
 */
public class BehaviorTreeNodeAddAndFreeze<T extends BehaviorTreeNode<T>> extends BehaviorTreeNodeAdd<T>
{
   public BehaviorTreeNodeAddAndFreeze(T nodeToAdd, T parent)
   {
      super(nodeToAdd, parent);
   }

   @Override
   public void performOperation()
   {
      super.performOperation();
      if (getParent() instanceof Freezable freezableParent)
         freezableParent.freezeFromModification();
   }
}
