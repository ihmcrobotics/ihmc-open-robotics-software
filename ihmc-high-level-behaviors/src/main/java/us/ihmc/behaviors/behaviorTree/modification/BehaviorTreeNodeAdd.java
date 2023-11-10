package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;

/**
 * Adds a node to a parent, without freezing its parent, for use in rebuilding a tree with
 * nodes that already existed.
 */
public class BehaviorTreeNodeAdd<T extends BehaviorTreeNode<T>> implements BehaviorTreeModification
{
   private final T nodeToAdd;
   private final T parent;

   public BehaviorTreeNodeAdd(T nodeToAdd, T parent)
   {
      this.nodeToAdd = nodeToAdd;
      this.parent = parent;
   }

   @Override
   public void performOperation()
   {
      parent.getChildren().add(nodeToAdd);
      nodeToAdd.setParent(parent);
   }

   protected BehaviorTreeNode<?> getNodeToAdd()
   {
      return nodeToAdd;
   }

   protected BehaviorTreeNode<?> getParent()
   {
      return parent;
   }
}
