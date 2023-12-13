package us.ihmc.rdx.ui.behavior.tree;

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
}
