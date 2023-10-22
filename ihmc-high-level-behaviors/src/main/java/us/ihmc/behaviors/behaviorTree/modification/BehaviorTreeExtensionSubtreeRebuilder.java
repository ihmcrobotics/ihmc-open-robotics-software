package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtensionSupplier;

import java.util.HashMap;

public class BehaviorTreeExtensionSubtreeRebuilder
{
   private final HashMap<Long, BehaviorTreeNodeExtension> idToNodesMap = new HashMap<>();

   private final BehaviorTreeModification clearSubtreeModification;
   private final BehaviorTreeModification destroyLeftoversModification;

   public BehaviorTreeExtensionSubtreeRebuilder(BehaviorTreeNodeExtensionSupplier subtreeNodeSupplier)
   {
      clearSubtreeModification = () -> clearChildren(subtreeNodeSupplier.getNodeExtension());

      destroyLeftoversModification = () ->
      {
         for (BehaviorTreeNodeExecutor leftover : idToNodesMap.values())
         {
            leftover.getState().destroy();
            leftover.destroy();
         }
      };
   }

   private void clearChildren(BehaviorTreeNodeExtension localNode)
   {
      idToNodesMap.put(localNode.getState().getID(), localNode);

      for (Object child : localNode.getChildren())
      {
         clearChildren((BehaviorTreeNodeExtension) child);
      }

      localNode.getDefinition().getChildren().clear();
      localNode.getState().getChildren().clear();
      localNode.getChildren().clear();
   }

   public BehaviorTreeNodeExtension getReplacementNode(long id)
   {
      return idToNodesMap.get(id);
   }

   public BehaviorTreeModification getReplacementModification(long id, BehaviorTreeNodeExtension parent)
   {
      return new BehaviorTreeNodeReplacement(idToNodesMap.remove(id), parent);
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

