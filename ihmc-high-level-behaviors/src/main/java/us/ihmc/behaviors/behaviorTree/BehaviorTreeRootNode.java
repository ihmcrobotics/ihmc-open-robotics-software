package us.ihmc.behaviors.behaviorTree;

/**
 * An interface for representing a root node for operations common to the UI and the robot.
 *
 * @param <T> The generic type of this node: RDX or Executor
 */
public interface BehaviorTreeRootNode<T extends BehaviorTreeNode<T, ?, ?>>
      extends BehaviorTreeNode<T, BehaviorTreeRootNodeState, BehaviorTreeRootNodeDefinition>
{

}
