package us.ihmc.behaviors.behaviorTree;

public interface BehaviorTreeNodeStateSupplier extends BehaviorTreeNodeDefinitionSupplier
{
   BehaviorTreeNodeState getState();
}
