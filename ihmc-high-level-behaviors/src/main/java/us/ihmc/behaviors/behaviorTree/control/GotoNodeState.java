package us.ihmc.behaviors.behaviorTree.control;

import behavior_msgs.msg.dds.GotoNodeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.LeafNodeState;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.List;

public class GotoNodeState extends LeafNodeState<GotoNodeDefinition>
{
   public GotoNodeState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new GotoNodeDefinition(rootNode.getDefinition()), rootNode);
   }

   @Override
   public void validateDefinition(List<BehaviorTreeNodeState<?>> nodes)
   {
      super.validateDefinition(nodes);

      if (definition.getNodeToGotoIsInvalid() || !rootNode.getIDToNodeMap().containsKey(definition.getNodeToGotoID()))
      {
         BehaviorTreeNodeState<?> bestNode = findBestNodeToGoto(definition.getNodeToGotoName());
         if (bestNode != null)
            definition.setNodeToGoto(bestNode.getID(), definition.getNodeToGotoName());
      }
      else if (definition.getNodeToGotoID() >= 0)
      {
         // Dynamically update the node name -- it can change independently of the node's ID
         // This is necessary for saving the definition
         for (BehaviorTreeNodeState<?> node : nodes)
         {
            if (node.getID() == definition.getNodeToGotoID())
            {
               definition.setNodeToGotoName(node.getDefinition().getName());
            }
         }
      }
   }

   @Override
   public void update()
   {
      super.update();
   }

   public void toMessage(GotoNodeStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(GotoNodeStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }

   public BehaviorTreeNodeState<?> findNodeToGoto()
   {
      BehaviorTreeNodeState<?> nodeToGoto = rootNode.getIDToNodeMap().get(definition.getNodeToGotoID());

      if (nodeToGoto == null)
         LogTools.error("Node to goto is not a leaf node.");

      return nodeToGoto;
   }

   // AI Generated after this

   private BehaviorTreeNodeState<?> findBestNodeToGoto(String name)
   {
      if (name == null)
         return null;

      BehaviorTreeNodeState<?> anchor = this;
      boolean includeAnchor = false;

      while (anchor != null)
      {
         BehaviorTreeNodeState<?> parent = anchor.getParent();
         if (parent == null)
         {
            if (includeAnchor && matchesName(anchor, name))
               return anchor;
            return null;
         }

         List<BehaviorTreeNodeState<?>> siblings = parent.getChildren();
         int anchorIndex = siblings.indexOf(anchor);

         BehaviorTreeNodeState<?> match = searchSiblingNodesByName(siblings, anchorIndex, anchor, includeAnchor, name);
         if (match != null)
            return match;

         match = searchSiblingSubtreesByName(siblings, anchorIndex, anchor, name);
         if (match != null)
            return match;

         anchor = parent;
         includeAnchor = true;
      }

      return null;
   }

   private BehaviorTreeNodeState<?> searchSiblingNodesByName(List<BehaviorTreeNodeState<?>> siblings,
                                                             int anchorIndex,
                                                             BehaviorTreeNodeState<?> anchor,
                                                             boolean includeAnchor,
                                                             String name)
   {
      if (includeAnchor && matchesName(anchor, name))
         return anchor;

      if (anchorIndex < 0)
      {
         for (BehaviorTreeNodeState<?> sibling : siblings)
         {
            if (sibling == anchor)
               continue;
            if (matchesName(sibling, name))
               return sibling;
         }
         return null;
      }

      int maxDistance = Math.max(anchorIndex, siblings.size() - anchorIndex - 1);
      for (int distance = 1; distance <= maxDistance; distance++)
      {
         int left = anchorIndex - distance;
         if (left >= 0)
         {
            BehaviorTreeNodeState<?> sibling = siblings.get(left);
            if (matchesName(sibling, name))
               return sibling;
         }

         int right = anchorIndex + distance;
         if (right < siblings.size())
         {
            BehaviorTreeNodeState<?> sibling = siblings.get(right);
            if (matchesName(sibling, name))
               return sibling;
         }
      }

      return null;
   }

   private BehaviorTreeNodeState<?> searchSiblingSubtreesByName(List<BehaviorTreeNodeState<?>> siblings,
                                                                int anchorIndex,
                                                                BehaviorTreeNodeState<?> anchor,
                                                                String name)
   {
      if (anchorIndex < 0)
      {
         for (BehaviorTreeNodeState<?> sibling : siblings)
         {
            if (sibling == anchor)
               continue;
            BehaviorTreeNodeState<?> match = searchSubtreeByName(sibling, name);
            if (match != null)
               return match;
         }
         return null;
      }

      int maxDistance = Math.max(anchorIndex, siblings.size() - anchorIndex - 1);
      for (int distance = 1; distance <= maxDistance; distance++)
      {
         int left = anchorIndex - distance;
         if (left >= 0)
         {
            BehaviorTreeNodeState<?> sibling = siblings.get(left);
            BehaviorTreeNodeState<?> match = searchSubtreeByName(sibling, name);
            if (match != null)
               return match;
         }

         int right = anchorIndex + distance;
         if (right < siblings.size())
         {
            BehaviorTreeNodeState<?> sibling = siblings.get(right);
            BehaviorTreeNodeState<?> match = searchSubtreeByName(sibling, name);
            if (match != null)
               return match;
         }
      }

      return null;
   }

   private final ArrayList<BehaviorTreeNodeState<?>> subtreeNodes = new ArrayList<>();

   private BehaviorTreeNodeState<?> searchSubtreeByName(BehaviorTreeNodeState<?> subtreeRoot, String name)
   {
      subtreeNodes.clear();
      List<BehaviorTreeNodeState<?>> children = subtreeRoot.getChildren();
      for (int i = children.size() - 1; i >= 0; i--)
         subtreeNodes.add(children.get(i));

      while (!subtreeNodes.isEmpty())
      {
         BehaviorTreeNodeState<?> node = subtreeNodes.remove(subtreeNodes.size() - 1);
         if (matchesName(node, name))
            return node;

         List<BehaviorTreeNodeState<?>> nodeChildren = node.getChildren();
         for (int i = nodeChildren.size() - 1; i >= 0; i--)
            subtreeNodes.add(nodeChildren.get(i));
      }

      return null;
   }

   private boolean matchesName(BehaviorTreeNodeState<?> node, String name)
   {
      return node != this && node.getDefinition().getName().equals(name);
   }
}
