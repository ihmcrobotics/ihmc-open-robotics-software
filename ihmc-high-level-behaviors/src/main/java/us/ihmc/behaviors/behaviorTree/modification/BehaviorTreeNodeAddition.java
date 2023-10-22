package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.communication.crdt.FreezableNode;

/**
 * Adds a node to a parent, freezing the parent if it's freezable.
 */
public class BehaviorTreeNodeAddition<T extends BehaviorTreeNode> implements BehaviorTreeModification<T>
{
   private final T nodeToAdd;
   private final T parent;

   public BehaviorTreeNodeAddition(T nodeToAdd, T parent)
   {
      this.nodeToAdd = nodeToAdd;
      this.parent = parent;
   }

   @Override
   public void performOperation()
   {
      parent.getChildren().add(nodeToAdd);
      if (parent instanceof FreezableNode freezableParent)
         freezableParent.freezeFromModification();
   }

   protected T getNodeToAdd()
   {
      return nodeToAdd;
   }

   protected T getParent()
   {
      return parent;
   }
}
