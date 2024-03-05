package us.ihmc.behaviors.behaviorTree;

import java.util.function.Consumer;

public class BehaviorTreeTools
{
   public static BehaviorTreeNodeDefinition findRootNode(BehaviorTreeNodeDefinition node)
   {
      while (!node.isRootNode())
         node = node.getParent();

      return node;
   }

   public static void runForSubtreeNodes(BehaviorTreeNodeDefinition node, Consumer<BehaviorTreeNodeDefinition> operation)
   {
      operation.accept(node);

      for (BehaviorTreeNodeDefinition child : node.getChildren())
      {
         runForSubtreeNodes(child, operation);
      }
   }

   public static void runForEntireTree(BehaviorTreeNodeDefinition anyNode, Consumer<BehaviorTreeNodeDefinition> operation)
   {
      runForSubtreeNodes(findRootNode(anyNode), operation);
   }
}
