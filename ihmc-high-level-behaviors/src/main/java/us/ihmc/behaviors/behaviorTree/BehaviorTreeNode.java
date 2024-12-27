package us.ihmc.behaviors.behaviorTree;

import javax.annotation.Nullable;
import java.util.List;

/**
 * Base interface for a behavior tree node. It is implemented by
 * several layers of node abstractions, including:
 * - Definition
 * - State
 * - Executor
 * - Graphical user interface
 *
 * A node has a list of children and a reference to the parent.
 *
 * @param <LT> The generic type of this node layer: UI, Executor, State, or Definition
 */
public interface BehaviorTreeNode<LT extends BehaviorTreeNode<LT>>
{
   /**
    * @return The node's children in order
    */
   List<LT> getChildren();

   /**
    * @param parent Sets the parent node or null if this is the root node
    */
   void setParent(@Nullable LT parent);

   /**
    * @return The parent node or null if this is the root node
    */
   @Nullable
   LT getParent();

   default boolean isRootNode()
   {
      return getParent() == null;
   }
}
