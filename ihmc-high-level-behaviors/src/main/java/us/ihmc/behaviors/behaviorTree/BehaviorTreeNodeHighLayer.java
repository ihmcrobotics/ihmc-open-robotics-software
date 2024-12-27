package us.ihmc.behaviors.behaviorTree;

import us.ihmc.log.LogTools;

/**
 * An interface that represents a behavior tree node's UI or Executor layer.
 *
 * @param <LT> The generic type of this node layer: UI or Executor
 * @param <S> The type of this node's state instance.
 * @param <D> The type of this node's definition instance.
 */
public interface BehaviorTreeNodeHighLayer<LT extends BehaviorTreeNodeHighLayer<LT, ?, ?>,
                                           S extends BehaviorTreeNodeState<D>,
                                           D extends BehaviorTreeNodeDefinition>
      extends BehaviorTreeNode<LT>
{
   S getState();

   D getDefinition();

   default void update()
   {
      getState().update();
   }

   default void destroy()
   {
      LogTools.info("Destroying node: {}:{}", getDefinition().getName(), getState().getID());
      getState().destroy();
   }
}
