package us.ihmc.behaviors.behaviorTree;

import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.ActionSequenceExecutor;
import us.ihmc.behaviors.sequence.ActionSequenceState;

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

   public static BehaviorTreeNode<?> findRootNodeGeneral(BehaviorTreeNode<?> node)
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

   public static ActionSequenceDefinition findActionSequenceAncestor(BehaviorTreeNodeDefinition node)
   {
      if (node instanceof ActionSequenceDefinition actionSequence)
      {
         return actionSequence;
      }
      else if (node == null || node.getParent() == null)
      {
         return null;
      }
      else if (node.getParent() instanceof ActionSequenceDefinition actionSequence)
      {
         return actionSequence;
      }
      else
      {
         return findActionSequenceAncestor(node.getParent());
      }
   }

   public static ActionSequenceState findActionSequenceAncestor(BehaviorTreeNodeState node)
   {
      if (node instanceof ActionSequenceState actionSequence)
      {
         return actionSequence;
      }
      else if (node == null || node.getParent() == null)
      {
         return null;
      }
      else if (node.getParent() instanceof ActionSequenceState actionSequence)
      {
         return actionSequence;
      }
      else
      {
         return findActionSequenceAncestor(node.getParent());
      }
   }

   public static ActionSequenceExecutor findActionSequenceAncestor(BehaviorTreeNodeExecutor node)
   {
      if (node instanceof ActionSequenceExecutor actionSequence)
      {
         return actionSequence;
      }
      else if (node == null || node.getParent() == null)
      {
         return null;
      }
      else if (node.getParent() instanceof ActionSequenceExecutor actionSequence)
      {
         return actionSequence;
      }
      else
      {
         return findActionSequenceAncestor(node.getParent());
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
}
