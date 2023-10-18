package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

import java.util.HashMap;

public class BehaviorTreeRebuildSubtree
{
   private final HashMap<Long, BehaviorTreeNodeState> idToNodesMap = new HashMap<>();

   private final BehaviorTreeModification clearSubtreeModification;
   private final BehaviorTreeModification destroyLeftoversModification;

   public BehaviorTreeRebuildSubtree(BehaviorTreeNodeState subtreeToRebuild)
   {
      clearSubtreeModification = () -> clearChildren(subtreeToRebuild);

      destroyLeftoversModification = () ->
      {
         removeRemainingFromMap(subtreeToRebuild);
         for (BehaviorTreeNodeState leftover : idToNodesMap.values())
         {
            leftover.destroy();
         }
      };
   }

   private void clearChildren(BehaviorTreeNodeState localNode)
   {
      idToNodesMap.put(localNode.getID(), localNode);

      for (BehaviorTreeNodeState child : localNode.getChildren())
      {
         clearChildren(child);
      }

      localNode.getChildren().clear();
   }

   private void removeRemainingFromMap(BehaviorTreeNodeState localNode)
   {
      idToNodesMap.remove(localNode.getID());

      for (BehaviorTreeNodeState child : localNode.getChildren())
      {
         removeRemainingFromMap(child);
      }
   }

   public BehaviorTreeModification getClearSubtreeModification()
   {
      return clearSubtreeModification;
   }

   public BehaviorTreeModification getDestroyLeftoversModification()
   {
      return destroyLeftoversModification;
   }
}

