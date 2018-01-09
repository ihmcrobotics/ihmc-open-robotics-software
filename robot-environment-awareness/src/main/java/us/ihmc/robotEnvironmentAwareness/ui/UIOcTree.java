package us.ihmc.robotEnvironmentAwareness.ui;

import java.util.Map;
import java.util.Map.Entry;

import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.ocTree.baseImplementation.AbstractOcTreeBase;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;

public class UIOcTree extends AbstractOcTreeBase<UIOcTreeNode>
{
   public UIOcTree(NormalOcTreeMessage normalOcTreeMessage)
   {
      this(normalOcTreeMessage, null);
   }

   public UIOcTree(NormalOcTreeMessage normalOcTreeMessage, Map<OcTreeKey, Integer> nodeKeyToRegionIdMap)
   {
      super(normalOcTreeMessage.resolution, normalOcTreeMessage.treeDepth);
      if (normalOcTreeMessage.root != null)
         root = new UIOcTreeNode(normalOcTreeMessage.root, resolution, treeDepth);
      else
         root = null;

      if (nodeKeyToRegionIdMap != null)
         nodeKeyToRegionIdMap.entrySet().stream().forEach(this::setNodeRegionId);

      if (root != null)
         updateInnerRegionIdRecursive(root, 0);
   }

   private void updateInnerRegionIdRecursive(UIOcTreeNode node, int depth)
   {
      // only recurse and update for inner nodes:
      if (node.hasAtLeastOneChild())
      {
         // return early for last level:
         if (depth < treeDepth - 1)
         {
            for (int i = 0; i < 8; i++)
            {
               UIOcTreeNode childNode = node.getChild(i);
               if (childNode != null)
                  updateInnerRegionIdRecursive(childNode, depth + 1);
            }
         }
         node.setRegionIdFromChildren();
      }
   }

   private void setNodeRegionId(Entry<OcTreeKey, Integer> nodeKeyToRegionIdEntry)
   {
      UIOcTreeNode node = search(nodeKeyToRegionIdEntry.getKey());
      if (node != null)
         node.setRegionId(nodeKeyToRegionIdEntry.getValue());
   }

   @Override
   protected Class<UIOcTreeNode> getNodeClass()
   {
      return UIOcTreeNode.class;
   }
}
