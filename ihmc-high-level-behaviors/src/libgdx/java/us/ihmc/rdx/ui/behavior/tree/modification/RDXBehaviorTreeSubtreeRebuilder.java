package us.ihmc.rdx.ui.behavior.tree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStateSupplier;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeRebuilder;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeStateModification;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;

import java.util.HashMap;

public class RDXBehaviorTreeSubtreeRebuilder implements BehaviorTreeRebuilder
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

   @Override
   public BehaviorTreeNodeStateSupplier getReplacementNode(long id)
   {
      return idToNodesMap.get(id);
   }

   @Override
   public BehaviorTreeStateModification getReplacementModification(long id)
   {
      return new RDXBehaviorTreeNodeReplacement(idToNodesMap.remove(id), subtreeToRebuild);
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

