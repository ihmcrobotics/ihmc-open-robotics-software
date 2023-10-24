package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;

/**
 * Adds a node to a parent, without freezing its parent, for use in rebuilding a tree with
 * nodes that already existed.
 */
public class BehaviorTreeNodeReplacement<T extends BehaviorTreeNode<T>> implements BehaviorTreeModification<T>
{
   private final T nodeToAdd;
   private final T parent;

   public BehaviorTreeNodeReplacement(T nodeToAdd, T parent)
   {
      this.nodeToAdd = nodeToAdd;
      this.parent = parent;
   }

   @Override
   public void performOperation()
   {
      parent.getChildren().add(nodeToAdd);
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
