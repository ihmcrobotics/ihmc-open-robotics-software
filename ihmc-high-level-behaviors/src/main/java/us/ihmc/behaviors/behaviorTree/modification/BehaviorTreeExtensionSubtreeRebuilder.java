package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtensionSupplier;
import us.ihmc.log.LogTools;

import java.util.HashMap;

public class BehaviorTreeExtensionSubtreeRebuilder
{
   private final HashMap<Long, BehaviorTreeNodeExtension<?, ?, ?, ?>> idToNodesMap = new HashMap<>();

   private final BehaviorTreeModification clearSubtreeModification;
   private final BehaviorTreeModification destroyLeftoversModification;

   public BehaviorTreeExtensionSubtreeRebuilder(BehaviorTreeNodeExtensionSupplier subtreeNodeSupplier)
   {
      clearSubtreeModification = () ->
      {
         idToNodesMap.clear();
         clearChildren(subtreeNodeSupplier.getNodeExtension());
      };

      destroyLeftoversModification = () ->
      {
         for (BehaviorTreeNodeExtension<?, ?, ?, ?> leftover : idToNodesMap.values())
         {
            LogTools.info("Destroying node: {}:{}", leftover.getState().getDefinition().getDescription(), leftover.getState().getID());
            leftover.getState().destroy();
            if (leftover.getExtendedNode() != leftover.getState()) // FIXME Kinda weird
               leftover.destroy();
         }
      };
   }

   private void clearChildren(BehaviorTreeNodeExtension<?, ?, ?, ?> localNode)
   {
      idToNodesMap.put(localNode.getState().getID(), localNode);

      if (!localNode.getState().isFrozen()) // Disassemble non-frozen parts
      {
         for (BehaviorTreeNode<?> child : localNode.getChildren())
         {
            clearChildren((BehaviorTreeNodeExtension<?, ?, ?, ?>) child);
         }

         localNode.getDefinition().getChildren().clear(); // FIXME This is kinda weird
         localNode.getState().getChildren().clear();
         localNode.getChildren().clear();
      }
   }

   public BehaviorTreeNodeExtension<?, ?, ?, ?> getReplacementNode(long id)
   {
      return idToNodesMap.remove(id);
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

