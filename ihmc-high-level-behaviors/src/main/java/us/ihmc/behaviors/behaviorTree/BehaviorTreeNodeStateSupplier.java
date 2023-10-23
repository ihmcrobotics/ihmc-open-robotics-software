package us.ihmc.behaviors.behaviorTree;

public interface BehaviorTreeNodeStateSupplier<S extends BehaviorTreeNodeState<S, D>,
                                               D extends BehaviorTreeNodeDefinition<D>>
      extends BehaviorTreeNodeDefinitionSupplier<D>
{
   S getState();
}
