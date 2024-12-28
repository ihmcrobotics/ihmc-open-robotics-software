package us.ihmc.behaviors.behaviorTree;

import us.ihmc.log.LogTools;

/**
 * An interface that represents a behavior tree node's RDX or Executor layer.
 *
 * @param <HLT> The generic type of this node high layer: RDX or Executor
 * @param <S> The type of this node's state instance.
 * @param <D> The type of this node's definition instance.
 */
public interface BehaviorTreeNodeHighLayer<HLT extends BehaviorTreeNodeHighLayer<HLT, ?, ?>,
                                           S extends BehaviorTreeNodeState<D>,
                                           D extends BehaviorTreeNodeDefinition>
      extends BehaviorTreeNode<HLT>
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
