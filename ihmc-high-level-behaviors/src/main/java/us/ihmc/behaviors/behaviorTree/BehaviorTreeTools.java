package us.ihmc.behaviors.behaviorTree;

import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class BehaviorTreeTools
{
   public static BehaviorTreeRootNodeExecutor findRootNode(BehaviorTreeNodeExecutor<?, ?> node)
   {
      return (BehaviorTreeRootNodeExecutor) findRootNodeGeneral(node);
   }

   public static BehaviorTreeRootNodeState findRootNode(BehaviorTreeNodeState<?> node)
   {
      return (BehaviorTreeRootNodeState) findRootNodeGeneral(node);
   }

   public static BehaviorTreeRootNodeDefinition findRootNode(BehaviorTreeNodeDefinition node)
   {
      return (BehaviorTreeRootNodeDefinition) findRootNodeGeneral(node);
   }

   public static <T extends BehaviorTreeNode<T, ?, ?>> T findRootNode(T node)
   {
      while (!node.isRootNode())
         node = node.getParent();

      return node;
   }

   public static TreeNode<?> findRootNodeGeneral(TreeNode<?> node)
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

   public static <T extends TreeNode<T>> void runForSubtreeNodes(T node, Consumer<T> operation)
   {
      operation.accept(node);

      for (T child : node.getChildren())
      {
         runForSubtreeNodes(child, operation);
      }
   }

   public static <T extends BehaviorTreeNode<T, ?, ?>> void runForSubtreeNodes(T node, Consumer<T> operation)
   {
      operation.accept(node);

      for (T child : node.getChildren())
      {
         runForSubtreeNodes(child, operation);
      }
   }

   public static List<ActionNodeDefinition> buildListOfActionDefinitions(BehaviorTreeNodeDefinition rootNode)
   {
      List<ActionNodeDefinition> actionDefinitions = new ArrayList<>();
      runForSubtreeNodes(rootNode, node ->
      {
         if (node instanceof ActionNodeDefinition actionNode)
         {
            actionDefinitions.add(actionNode);
         }
      });
      return actionDefinitions;
   }

   public static int getNodeDepth(BehaviorTreeNodeState<?> node)
   {
      int depth = 0;
      while (!node.isRootNode())
      {
         ++depth;
         node = node.getParent();
      }
      return depth;
   }

   public static int getChildIndex(BehaviorTreeNodeState<?> node)
   {
      return node.isRootNode() ? 0 : node.getParent().getChildren().indexOf(node);
   }
}
