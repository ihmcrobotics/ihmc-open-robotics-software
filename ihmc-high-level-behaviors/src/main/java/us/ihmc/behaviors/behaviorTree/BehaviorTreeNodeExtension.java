package us.ihmc.behaviors.behaviorTree;

public interface BehaviorTreeNodeExtension<T extends BehaviorTreeNode<T>,
                                           U extends BehaviorTreeNode<U>> extends BehaviorTreeNode<T>, BehaviorTreeNodeStateSupplier
{
   U getExtendedNode();

   void destroy();
}
