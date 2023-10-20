package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStateSupplier;

import java.util.HashMap;

public class BehaviorTreeExecutorSubtreeRebuilder implements BehaviorTreeRebuilder
{
   private final BehaviorTreeNodeExecutor subtreeToRebuild;

   private final HashMap<Long, BehaviorTreeNodeExecutor> idToNodesMap = new HashMap<>();

   private final BehaviorTreeExecutorModification clearSubtreeModification;
   private final BehaviorTreeExecutorModification destroyLeftoversModification;

   public BehaviorTreeExecutorSubtreeRebuilder(BehaviorTreeNodeExecutor subtreeToRebuild)
   {
      this.subtreeToRebuild = subtreeToRebuild;

      clearSubtreeModification = () -> clearChildren(subtreeToRebuild);

      destroyLeftoversModification = () ->
      {
         for (BehaviorTreeNodeExecutor leftover : idToNodesMap.values())
         {
            leftover.getState().destroy();
            leftover.destroy();
         }
      };
   }

   private void clearChildren(BehaviorTreeNodeExecutor localNode)
   {
      idToNodesMap.put(localNode.getState().getID(), localNode);

      for (BehaviorTreeNodeExecutor child : localNode.getChildren())
      {
         clearChildren(child);
      }

      localNode.getDefinition().getChildren().clear();
      localNode.getState().getChildren().clear();
      localNode.getChildren().clear();
   }

   @Override
   public BehaviorTreeNodeStateSupplier getReplacementNode(long id)
   {
      return idToNodesMap.get(id);
   }

   @Override
   public BehaviorTreeStateModification getReplacementModification(long id)
   {
      return new BehaviorTreeExecutorNodeReplacement(idToNodesMap.remove(id), subtreeToRebuild);
   }

   @Override
   public BehaviorTreeStateModification getClearSubtreeModification()
   {
      return clearSubtreeModification;
   }

   @Override
   public BehaviorTreeStateModification getDestroyLeftoversModification()
   {
      return destroyLeftoversModification;
   }
}

