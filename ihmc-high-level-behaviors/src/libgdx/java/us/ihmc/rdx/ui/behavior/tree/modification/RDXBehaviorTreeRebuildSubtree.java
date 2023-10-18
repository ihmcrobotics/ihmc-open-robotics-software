package us.ihmc.rdx.ui.behavior.tree.modification;

import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeStateRebuildSubtree;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;

import java.util.HashMap;

public class RDXBehaviorTreeRebuildSubtree
{
   private final BehaviorTreeStateRebuildSubtree stateRebuildSubtree;

   private final HashMap<Long, RDXBehaviorTreeNode> idToNodesMap = new HashMap<>();

   private final RDXBehaviorTreeModification clearSubtreeModification;
   private final RDXBehaviorTreeModification destroyLeftoversModification;

   public RDXBehaviorTreeRebuildSubtree(RDXBehaviorTreeNode subtreeToRebuild)
   {
      stateRebuildSubtree = new BehaviorTreeStateRebuildSubtree(subtreeToRebuild.getState());

      clearSubtreeModification = () ->
      {
         stateRebuildSubtree.getClearSubtreeModification().performOperation();
         clearChildren(subtreeToRebuild);
      };

      destroyLeftoversModification = () ->
      {
         stateRebuildSubtree.getDestroyLeftoversModification().performOperation();

         removeRemainingFromMap(subtreeToRebuild);

         for (RDXBehaviorTreeNode leftover : idToNodesMap.values())
         {
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

      localNode.getChildren().clear();
   }

   private void removeRemainingFromMap(RDXBehaviorTreeNode localNode)
   {
      idToNodesMap.remove(localNode.getState().getID());

      for (RDXBehaviorTreeNode child : localNode.getChildren())
      {
         removeRemainingFromMap(child);
      }
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

