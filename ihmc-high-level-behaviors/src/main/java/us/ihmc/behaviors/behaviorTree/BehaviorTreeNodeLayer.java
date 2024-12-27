package us.ihmc.behaviors.behaviorTree;

import us.ihmc.tools.Destroyable;

/**
 * This is an interface that represents a behavior tree node's
 * State, UI, or Executor layer.
 *
 * A "Layer" will be RDX, Executor, State, where Definition is always the base layer.
 *
 * The layers actually do not extend each other, they are more encapsulations of the
 * lower layers that add functionality. A node exists through it's state and different
 * processes may wrap that synchronized state with layers appropriate for acting in that process.
 *
 * TODO: Consider collapsing this away.
 *
 * @param <LT> This node's layer type. It will be a State, RDX, or Executor layer type.
 *            This is so we can extend BehaviorTreeNode and provide the appropriate
 *            base type of children.
 * @param <S> The type of this node's state instance.
 * @param <D> The type of this node's definition instance.
 */
public interface BehaviorTreeNodeLayer<LT extends BehaviorTreeNode<LT>,
                                       S extends BehaviorTreeNodeState<S, D>,
                                       D extends BehaviorTreeNodeDefinition>
      extends BehaviorTreeNode<LT>, Destroyable
{

   S getState();

   D getDefinition();

   default void update()
   {
      // Do nothing
   }
}
