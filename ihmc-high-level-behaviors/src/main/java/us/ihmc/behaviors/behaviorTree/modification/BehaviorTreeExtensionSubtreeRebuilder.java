package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtensionSupplier;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.log.LogTools;

import java.util.HashMap;

public class BehaviorTreeExtensionSubtreeRebuilder
{
   private final HashMap<Long, BehaviorTreeNodeExtension<?, ?, ?, ?>> idToNodesMap = new HashMap<>();
   private final CRDTInfo crdtInfo;

   private final BehaviorTreeModification clearSubtreeModification;
   private final BehaviorTreeModification destroyLeftoversModification;

   public BehaviorTreeExtensionSubtreeRebuilder(BehaviorTreeNodeExtensionSupplier subtreeNodeSupplier, CRDTInfo crdtInfo)
   {
      this.crdtInfo = crdtInfo;

      clearSubtreeModification = () ->
      {
         if (!idToNodesMap.isEmpty())
         {
            LogTools.error("Clearing {} nodes in map. Actor: {}", idToNodesMap.size(), crdtInfo.getActorDesignation());
            idToNodesMap.clear();
         }
         clearChildren(subtreeNodeSupplier.getNodeExtension());
      };

      destroyLeftoversModification = () ->
      {
         for (BehaviorTreeNodeExtension<?, ?, ?, ?> leftover : idToNodesMap.values())
         {
            leftover.destroy();
         }
         idToNodesMap.clear();
      };
   }

   private void clearChildren(BehaviorTreeNodeExtension<?, ?, ?, ?> localNode)
   {
      if (localNode != null) // In the case of a null root node
      {
//         LogTools.info("Putting ID: %s:%d. Children: %d Actor: %s".formatted(localNode.getDefinition().getDescription(),
//                                                                             localNode.getState().getID(),
//                                                                             localNode.getChildren().size(),
//                                                                             crdtInfo.getActorDesignation()));
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
         else
         {
//            LogTools.info("Rejecting frozen %s:%d. Children: %d Actor: %s".formatted(localNode.getDefinition().getDescription(),
//                                                                                    localNode.getState().getID(),
//                                                                                    localNode.getChildren().size(),
//                                                                                    crdtInfo.getActorDesignation()));
         }
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

