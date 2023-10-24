package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.communication.crdt.Freezable;

/**
 * Searches the tree, removing the given node and freezing it's parent.
 */
public class BehaviorTreeNodeRemove<T extends BehaviorTreeNode<T>> implements BehaviorTreeModification
{
   private final T nodeToRemove;
   private final T rootNode;

   public BehaviorTreeNodeRemove(T nodeToRemove, T rootNode)
   {
      this.nodeToRemove = nodeToRemove;
      this.rootNode = rootNode;
   }

   @Override
   public void performOperation()
   {
      findAndRemove(rootNode);
   }

   private void findAndRemove(T parentNode)
   {
      if (parentNode.getChildren().remove(nodeToRemove))
      {
         if (parentNode instanceof Freezable freezableNode)
         {
            freezableNode.freezeFromModification();
         }
      }
      else
      {
         for (T child : parentNode.getChildren())
         {
            findAndRemove(child);
         }
      }
   }
}
