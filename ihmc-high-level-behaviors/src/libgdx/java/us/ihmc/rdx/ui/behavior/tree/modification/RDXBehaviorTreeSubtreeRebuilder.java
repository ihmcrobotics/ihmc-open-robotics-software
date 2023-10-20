package us.ihmc.rdx.ui.behavior.tree.modification;

import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;

import java.util.HashMap;

public class RDXBehaviorTreeSubtreeRebuilder
{
   private final RDXBehaviorTreeNode subtreeToRebuild;

   private final HashMap<Long, RDXBehaviorTreeNode> idToNodesMap = new HashMap<>();

   private final RDXBehaviorTreeModification clearSubtreeModification;
   private final RDXBehaviorTreeModification destroyLeftoversModification;

   public RDXBehaviorTreeSubtreeRebuilder(RDXBehaviorTreeNode subtreeToRebuild)
   {
      this.subtreeToRebuild = subtreeToRebuild;

      clearSubtreeModification = () -> clearChildren(subtreeToRebuild);

      destroyLeftoversModification = () ->
      {
         for (RDXBehaviorTreeNode leftover : idToNodesMap.values())
         {
            leftover.getState().destroy();
            leftover.destroy();
         }
      };
   }

   private void clearChildren(RDXBehaviorTreeNode localNode)
   {
      idToNodesMap.put(localNode.getState().getID(), localNode);

      for (RDXBehaviorTreeNode child : localNode.getChildren())
      {
         clearChildren(child);
      }

      localNode.getDefinition().getChildren().clear();
      localNode.getState().getChildren().clear();
      localNode.getChildren().clear();
   }

   public RDXBehaviorTreeNodeReplacement createReplacement(long id)
   {
      return new RDXBehaviorTreeNodeReplacement(idToNodesMap.remove(id), subtreeToRebuild);
   }

   public RDXBehaviorTreeModification getClearSubtreeModification()
   {
      return clearSubtreeModification;
   }

   public RDXBehaviorTreeModification getDestroyLeftoversModification()
   {
      return destroyLeftoversModification;
   }
}

