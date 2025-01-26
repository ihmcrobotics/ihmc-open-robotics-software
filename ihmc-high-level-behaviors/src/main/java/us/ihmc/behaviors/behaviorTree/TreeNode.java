package us.ihmc.behaviors.behaviorTree;

import javax.annotation.Nullable;
import java.util.List;

/**
 * The base interface for a tree node with a list of children and a reference to the parent.
 *
 * @param <T> The generic type of this node.
 */
public interface TreeNode<T extends TreeNode<T>>
{
   /**
    * @return The node's children in order
    */
   List<T> getChildren();

   /**
    * @param parent Sets the parent node or null if this is the root node
    */
   void setParent(@Nullable T parent);

   /**
    * @return The parent node or null if this is the root node
    */
   @Nullable
   T getParent();

   default boolean isRootNode()
   {
      return getParent() == null;
   }
}
