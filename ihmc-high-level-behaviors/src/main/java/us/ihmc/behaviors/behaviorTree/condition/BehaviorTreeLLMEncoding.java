package us.ihmc.behaviors.behaviorTree.condition;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.control.ActionSequenceState;
import us.ihmc.behaviors.behaviorTree.LeafNodeState;
import us.ihmc.log.LogTools;

public class BehaviorTreeLLMEncoding
{
   public static String encode(BehaviorTreeRootNodeState rootNode)
   {
      StringBuilder builder = new StringBuilder();
      
      builder.append("nodes: [\n");
      
      encodeTree(rootNode, builder, 0);

      builder.append(" ],%nstate: { execution_next_index: %d }".formatted(rootNode.getExecutionNextIndex()));
      
      return builder.toString();
   }

   private static void encodeTree(BehaviorTreeNodeState<?> node, StringBuilder builder, int indent)
   {
      builder.append("\t".repeat(indent));

      if (node instanceof LeafNodeState<?> leafNode)
      {
         builder.append("{ type: leaf, index: %d, is_executing: %b, failed: %b, can_execute: %b }"
                              .formatted(leafNode.getLeafIndex(),
                                         leafNode.getIsExecuting(),
                                         leafNode.getFailed(),
                                         leafNode.getCanExecute()));
      }
      else if (node instanceof ActionSequenceState sequenceNode)
      {
         builder.append("{ type: sequence, children: [\n");

         for (BehaviorTreeNodeState<?> child : node.getChildren())
         {
            encodeTree(child, builder, indent + 1);
            builder.append("\n");
         }

         builder.append("\t".repeat(indent));

         builder.append("]");
         builder.append(" }");
      }
      else
      {
         LogTools.error("Implement node type: " + node.getClass().getSimpleName());

         for (BehaviorTreeNodeState<?> child : node.getChildren())
         {
            encodeTree(child, builder, indent);
         }
      }

   }
}
