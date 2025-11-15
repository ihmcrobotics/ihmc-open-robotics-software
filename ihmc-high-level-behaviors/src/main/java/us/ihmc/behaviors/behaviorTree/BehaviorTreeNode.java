package us.ihmc.behaviors.behaviorTree;

import us.ihmc.log.LogTools;
import us.ihmc.tools.Destroyable;

/**
 * An interface that represents a complete behavior tree node for instantiation in
 * the RDX UI or on the robot as an executor. It encapsulates the node state and definition
 * while allowing extension to implement the various node types.
 *
 * @param <T> The generic type of this node: RDX or Executor
 * @param <S> The type of this node's state instance.
 * @param <D> The type of this node's definition instance.
 */
public interface BehaviorTreeNode<T extends BehaviorTreeNode<T, ?, ?>,
                                  S extends BehaviorTreeNodeState<D>,
                                  D extends BehaviorTreeNodeDefinition>
      extends TreeNode<T>, Destroyable
{
   S getState();

   D getDefinition();

   /** Update the node's state. Should not have side effects if called multiple times per tick. */
   default void update()
   {
      getState().update();
   }

   @Override
   default void destroy()
   {
      LogTools.info("{}: Destroying node: {}:{}", getClass().getSimpleName(), getDefinition().getName(), getState().getID());
      getState().destroy();
   }
}
