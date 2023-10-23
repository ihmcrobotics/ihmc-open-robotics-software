package us.ihmc.behaviors.behaviorTree;

public interface BehaviorTreeNodeDefinitionSupplier<D extends BehaviorTreeNodeDefinition<D>>
{
   D getDefinition();
}
