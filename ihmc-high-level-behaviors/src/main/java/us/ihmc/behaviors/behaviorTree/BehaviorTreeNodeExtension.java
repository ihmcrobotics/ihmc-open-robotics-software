package us.ihmc.behaviors.behaviorTree;

/**
 * @param <T> This node's type. It will be a State, RDX, or Executor type.
 * @param <E> The type of the extended part of the node, State or Definition
 */
public interface BehaviorTreeNodeExtension<T extends BehaviorTreeNode<T>,
                                           E extends BehaviorTreeNode<E>,
                                           D extends BehaviorTreeNodeDefinition<D>,
                                           S extends BehaviorTreeNodeState<S, D>>
      extends BehaviorTreeNode<T>,
              BehaviorTreeNodeStateSupplier<S, D>
{
   E getExtendedNode();

   void destroy();
}
