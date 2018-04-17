package us.ihmc.robotEnvironmentAwareness.communication.converters;

import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeNodeMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.OcTreeKeyMessage;

public class OcTreeMessageConverter
{
   public static NormalOcTreeMessage convertToMessage(NormalOcTree normalOcTree)
   {
      NormalOcTreeMessage normalOcTreeMessage = new NormalOcTreeMessage();

      normalOcTreeMessage.resolution = (float) normalOcTree.getResolution();
      normalOcTreeMessage.treeDepth = normalOcTree.getTreeDepth();

      if (normalOcTree.getRoot() != null)
      {
         normalOcTreeMessage.root = new NormalOcTreeNodeMessage();
         fullDepthCopy(normalOcTree.getRoot(), normalOcTreeMessage.root);
      }

      return normalOcTreeMessage;
   }

   private static void fullDepthCopy(NormalOcTreeNode nodeOriginal, NormalOcTreeNodeMessage nodeCopy)
   {
      nodeCopy.depth = nodeOriginal.getDepth();
      nodeCopy.size = (float) nodeOriginal.getSize();

      nodeCopy.key0 = nodeOriginal.getKey0();
      nodeCopy.key1 = nodeOriginal.getKey1();
      nodeCopy.key2 = nodeOriginal.getKey2();

      nodeCopy.normalX = (float) nodeOriginal.getNormalX();
      nodeCopy.normalY = (float) nodeOriginal.getNormalY();
      nodeCopy.normalZ = (float) nodeOriginal.getNormalZ();

      nodeCopy.normalAverageDeviation = nodeCopy.normalAverageDeviation;
      nodeCopy.normalConsensusSize = nodeCopy.normalConsensusSize;

      nodeCopy.hitLocationX = (float) nodeOriginal.getHitLocationX();
      nodeCopy.hitLocationY = (float) nodeOriginal.getHitLocationY();
      nodeCopy.hitLocationZ = (float) nodeOriginal.getHitLocationZ();

      nodeCopy.numberOfHits = nodeOriginal.getNumberOfHits();

      if (nodeOriginal.hasAtLeastOneChild())
      {
         nodeCopy.children = new NormalOcTreeNodeMessage[8];
         for (int childIndex = 0; childIndex < 8; childIndex++)
         {
            NormalOcTreeNode childOriginal = nodeOriginal.getChild(childIndex);
            if (childOriginal == null)
               continue;

            NormalOcTreeNodeMessage childCopy = new NormalOcTreeNodeMessage();
            nodeCopy.children[childIndex] = childCopy;
            fullDepthCopy(childOriginal, childCopy);
         }
      }
   }

   public static OcTreeKeyMessage createOcTreeKeyMessage(int k0, int k1, int k2)
   {
      OcTreeKeyMessage message = new OcTreeKeyMessage();
      if (message.k == null)
         message.k = new int[3];
      message.k[0] = k0;
      message.k[1] = k1;
      message.k[2] = k2;
      return message;
   }

   public static OcTreeKeyMessage createOcTreeKeyMessage(OcTreeKeyReadOnly other)
   {
      return createOcTreeKeyMessage(other.getKey(0), other.getKey(1), other.getKey(2));
   }
}
