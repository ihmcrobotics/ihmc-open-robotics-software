package us.ihmc.behaviors.behaviorTree;

/**
 * An interface that represents a behavior tree root node's RDX or Executor layer.
 *
 * @param <T> The type of this node's high layer instance.
 * @param <HLT> The generic type of this node high layer: RDX or Executor
 * @param <S> The type of this node's state instance.
 * @param <D> The type of this node's definition instance.
 */
public interface BehaviorTreeRootNode<T extends BehaviorTreeRootNode<T, HLT, S, D>,
                                      HLT extends BehaviorTreeNodeHighLayer<HLT, ?, ?>,
                                      S extends BehaviorTreeNodeState<D>,
                                      D extends BehaviorTreeNodeDefinition>
      extends BehaviorTreeNodeHighLayer<HLT, S, D>
{

}
