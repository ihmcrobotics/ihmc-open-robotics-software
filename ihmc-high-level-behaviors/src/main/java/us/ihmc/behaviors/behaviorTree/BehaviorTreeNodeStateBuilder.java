package us.ihmc.behaviors.behaviorTree;

/**
 * States won't be built directly. They'll be Executors or UI types.
 */
public interface BehaviorTreeNodeStateBuilder
{
   BehaviorTreeNodeStateSupplier createNode(Class<?> nodeType, long id);
}
