package us.ihmc.behaviors.behaviorTree;

/**
 * A builder interface used for common code in building nodes in the UI and on the robot.
 *
 * @param <T> The generic type of this node: RDX or Executor
 */
public interface BehaviorTreeNodeBuilder<T extends BehaviorTreeNode<T, ? ,?>>
{
   BehaviorTreeRootNode<T> createRootNode(long id);

   T createNode(Class<?> nodeType, long id, BehaviorTreeRootNode<T> rootNode);
}
