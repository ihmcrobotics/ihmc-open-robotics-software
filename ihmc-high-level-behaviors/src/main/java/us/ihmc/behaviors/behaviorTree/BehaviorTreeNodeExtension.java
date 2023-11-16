package us.ihmc.behaviors.behaviorTree;

import us.ihmc.tools.Destroyable;

/**
 * @param <T> This node's type. It will be a State, RDX, or Executor type.
 *            This is so we can extend BehaviorTreeNode and provide the appropriate
 *            base type of children.
 * @param <E> The type of the extended part of the node, State or Definition
 * @param <S> The type of the state of this node.
 * @param <D> The type of the definition of this node.
 */
public interface BehaviorTreeNodeExtension<T extends BehaviorTreeNode<T>,
                                           E extends BehaviorTreeNode<?>,
                                           S extends BehaviorTreeNodeState<D>,
                                           D extends BehaviorTreeNodeDefinition>
      extends BehaviorTreeNode<T>, Destroyable
{
   /**
    * @return A node of type State or Definition
    */
   E getExtendedNode();

   S getState();

   D getDefinition();

   default void update()
   {
      // Update state only if this instance extends State
      if (getExtendsState())
         getState().update();
   }

   default boolean getExtendsState()
   {
      return getState() == getExtendedNode();
   }
}
