package us.ihmc.rdx.ui.behavior.tree;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;

import java.util.function.Consumer;

public class RDXBehaviorTreeTools
{
   public static RDXBehaviorTreeRootNode findRootNode(RDXBehaviorTreeNode<?, ?> node)
   {
      return (RDXBehaviorTreeRootNode) BehaviorTreeTools.findRootNodeGeneral(node);
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
