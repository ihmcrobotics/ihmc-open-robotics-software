package us.ihmc.behaviors.behaviorTree.topology;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeLayer;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeLayerSupplier;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.log.LogTools;

import java.util.HashMap;

public class BehaviorTreeExtensionSubtreeRebuilder
{
   private final HashMap<Long, BehaviorTreeNodeLayer<?, ?, ?, ?>> idToNodesMap = new HashMap<>();
   private final CRDTInfo crdtInfo;

   private final BehaviorTreeTopologyOperation clearSubtreeOperation;
   private final BehaviorTreeTopologyOperation destroyLeftoversOperation;

   public BehaviorTreeExtensionSubtreeRebuilder(BehaviorTreeNodeLayerSupplier subtreeNodeSupplier, CRDTInfo crdtInfo)
   {
      this.crdtInfo = crdtInfo;

      clearSubtreeOperation = () ->
      {
         if (!idToNodesMap.isEmpty())
         {
            LogTools.error("Clearing {} nodes in map. Actor: {}", idToNodesMap.size(), crdtInfo.getActorDesignation());
            idToNodesMap.clear();
         }
         clearChildren(subtreeNodeSupplier.getNodeLayer());
      };

      destroyLeftoversOperation = () ->
      {
         for (BehaviorTreeNodeLayer<?, ?, ?, ?> leftover : idToNodesMap.values())
         {
            leftover.destroy();
         }
         idToNodesMap.clear();
      };
   }

   private void clearChildren(BehaviorTreeNodeLayer<?, ?, ?, ?> localNode)
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
               clearChildren((BehaviorTreeNodeLayer<?, ?, ?, ?>) child);
            }

            BehaviorTreeTopologyOperations.clearChildren(localNode);
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

   public BehaviorTreeNodeLayer<?, ?, ?, ?> getReplacementNode(long id)
   {
      return idToNodesMap.remove(id);
   }

   public BehaviorTreeTopologyOperation getClearSubtreeOperation()
   {
      return clearSubtreeOperation;
   }

   public BehaviorTreeTopologyOperation getDestroyLeftoversOperation()
   {
      return destroyLeftoversOperation;
   }
}

