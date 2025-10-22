package us.ihmc.behaviors.behaviorTree;

import us.ihmc.tools.io.WorkspaceResourceDirectory;

/**
 * A builder interface used for common code in building nodes in the UI and on the robot.
 *
 * @param <R> The type of root node: RDX or Executor
 * @param <T> The generic type of this node: RDX or Executor
 */
public interface BehaviorTreeNodeBuilder<R extends BehaviorTreeRootNode<T>, T extends BehaviorTreeNode<T, ? ,?>>
{
   T createNode(Class<?> nodeType, long id, R rootNode, WorkspaceResourceDirectory saveFileDirectory);
}
