package us.ihmc.behaviors.behaviorTree;

public interface BehaviorTreeNodeExtension<T extends BehaviorTreeNode<T>,
                                           U extends BehaviorTreeNode<U>> extends BehaviorTreeNode<T>
{
   U getExtendedNode();

   void destroy();
}
