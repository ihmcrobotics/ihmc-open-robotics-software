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
 * @param <T> This node's type. It will be a State, RDX, or Executor layer type.
 *            This is so we can extend BehaviorTreeNode and provide the appropriate
 *            base type of children.
 * @param <E> The type of the next lower layer which will be the State or Definition layer.
 * @param <S> The type of the state layer of this node.
 * @param <D> The type of the definition layer of this node.
 */
public interface BehaviorTreeNodeLayer<T extends BehaviorTreeNode<T>,
                                       E extends BehaviorTreeNode<?>,
                                       S extends BehaviorTreeNodeState<D>,
                                       D extends BehaviorTreeNodeDefinition>
      extends BehaviorTreeNode<T>, Destroyable
{
   /**
    * @return A node of type State or Definition
    */
   E getNextLowerLayer();

   S getState();

   D getDefinition();

   default void update()
   {
      // Update state only if this layer is over the State layer
      if (isLayerOverState())
         getState().update();
   }

   default boolean isLayerOverState()
   {
      return getState() == getNextLowerLayer();
   }
}
