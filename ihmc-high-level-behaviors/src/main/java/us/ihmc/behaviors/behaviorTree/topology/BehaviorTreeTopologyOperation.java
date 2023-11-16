package us.ihmc.behaviors.behaviorTree.topology;

/**
 * Describes and has everything needed to perform a modification of
 * the behavior tree structure (topology). This is so they can be queued up
 * and performed inside the scene graph with consistency guarantees.
 */
@FunctionalInterface
public interface BehaviorTreeTopologyOperation
{
   void performOperation();
}
