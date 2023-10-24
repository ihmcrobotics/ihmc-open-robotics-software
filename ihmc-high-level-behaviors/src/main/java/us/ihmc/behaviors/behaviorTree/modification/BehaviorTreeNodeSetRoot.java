package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;

import java.util.function.Consumer;

/**
 * Replaces the root node of the tree with a new one.
 */
public class BehaviorTreeNodeSetRoot<T extends BehaviorTreeNode<T>> implements BehaviorTreeModification
{
   private final T newRoot;
   private final Consumer<T> rootSetter;

   public BehaviorTreeNodeSetRoot(T newRoot, Consumer<T> rootSetter)
   {
      this.newRoot = newRoot;
      this.rootSetter = rootSetter;
   }

   @Override
   public void performOperation()
   {
      rootSetter.accept(newRoot);
   }
}
