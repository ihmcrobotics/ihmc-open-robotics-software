package us.ihmc.rdx.ui.behavior.tree;

import us.ihmc.rdx.ui.behavior.sequence.RDXActionSequence;

import java.util.function.Consumer;

public class RDXBehaviorTreeTools
{
   public static RDXBehaviorTreeNode<?, ?> findRootNode(RDXBehaviorTreeNode<?, ?> node)
   {
      while (!node.isRootNode())
         node = node.getParent();

      return node;
   }

   public static void runForSubtreeNodes(RDXBehaviorTreeNode<?, ?> node, Consumer<RDXBehaviorTreeNode<?, ?>> operation)
   {
      operation.accept(node);

      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         runForSubtreeNodes(child, operation);
      }
   }

   public static void runForEntireTree(RDXBehaviorTreeNode<?, ?> anyNode, Consumer<RDXBehaviorTreeNode<?, ?>> operation)
   {
      runForSubtreeNodes(findRootNode(anyNode), operation);
   }

   public static RDXActionSequence findActionSequenceAncestor(RDXBehaviorTreeNode<?, ?> node)
   {
      if (node instanceof RDXActionSequence actionSequence)
      {
         return actionSequence;
      }
      else if (node == null || node.getParent() == null)
      {
         return null;
      }
      else if (node.getParent() instanceof RDXActionSequence actionSequence)
      {
         return actionSequence;
      }
      else
      {
         return findActionSequenceAncestor(node.getParent());
      }
   }

   public static void clearOtherNodeSelections(RDXBehaviorTreeNode<?, ?> anyNode)
   {
      runForEntireTree(anyNode, node ->
      {
         if (node != anyNode)
         {
            node.clearSelections();
         }
      });
   }
}
