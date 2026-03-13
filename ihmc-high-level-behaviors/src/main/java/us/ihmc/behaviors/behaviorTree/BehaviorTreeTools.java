package us.ihmc.behaviors.behaviorTree;

import us.ihmc.behaviors.behaviorTree.action.ActionNodeDefinition;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

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

   public static BehaviorTreeNodeExecutor<?, ?> searchDFSFirstMatch(BehaviorTreeNodeExecutor<?, ?> node,
                                                                    Function<BehaviorTreeNodeExecutor<?, ?>, Boolean> predicate)
   {
      if (predicate.apply(node))
         return node;

      for (BehaviorTreeNodeExecutor<?, ?> child : node.getChildren())
      {
         BehaviorTreeNodeExecutor<?, ?> result = searchDFSFirstMatch(child, predicate);
         if (result != null)
            return result;
      }

      return null;
   }

   public static BehaviorTreeNodeExecutor<?, ?> searchDFSFirstMatch(BehaviorTreeNodeExecutor<?, ?> node, String name)
   {
      if (node.getDefinition().getName().equals(name))
         return node;

      for (BehaviorTreeNodeExecutor<?, ?> child : node.getChildren())
      {
         BehaviorTreeNodeExecutor<?, ?> result = searchDFSFirstMatch(child, name);
         if (result != null)
            return result;
      }

      return null;
   }

   public static <T extends BehaviorTreeNode<T, ?, ?>> int getNodeIndexDFS(T node)
   {
      Deque<T> stack = new ArrayDeque<>();

      T rootNode = node;
      while (!rootNode.isRootNode())
         rootNode = rootNode.getParent();

      stack.push(rootNode);
      int index = 0;
      while (!stack.isEmpty())
      {
         T current = stack.pop();

         if (current == node)
            break;

         index++;

         // Push children in reverse order so the first child is processed first
         List<T> children = current.getChildren();
         for (int i = children.size() - 1; i >= 0; i--)
            stack.push(children.get(i));
      }

      return index;
   }

   public static int getChildIndex(BehaviorTreeNodeState<?> node)
   {
      return node.isRootNode() ? 0 : node.getParent().getChildren().indexOf(node);
   }
}
