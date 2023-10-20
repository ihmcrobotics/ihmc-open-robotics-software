package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;

import java.util.HashMap;

public class BehaviorTreeStateSubtreeRebuilder
{
   private final BehaviorTreeNodeState subtreeToRebuild;

   private final HashMap<Long, BehaviorTreeNodeState> idToNodesMap = new HashMap<>();

   private final BehaviorTreeStateModification clearSubtreeModification;
   private final BehaviorTreeStateModification destroyLeftoversModification;

   public BehaviorTreeStateSubtreeRebuilder(BehaviorTreeNodeState subtreeToRebuild)
   {
      this.subtreeToRebuild = subtreeToRebuild;

      clearSubtreeModification = () -> clearChildren(subtreeToRebuild);

      destroyLeftoversModification = () ->
      {
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

      localNode.getDefinition().getChildren().clear();
      localNode.getChildren().clear();
   }

   public BehaviorTreeStateNodeReplacement createReplacement(long id)
   {
      return new BehaviorTreeStateNodeReplacement(idToNodesMap.remove(id), subtreeToRebuild);
   }

   public BehaviorTreeStateModification getClearSubtreeModification()
   {
      return clearSubtreeModification;
   }

   public BehaviorTreeStateModification getDestroyLeftoversModification()
   {
      return destroyLeftoversModification;
   }
}

