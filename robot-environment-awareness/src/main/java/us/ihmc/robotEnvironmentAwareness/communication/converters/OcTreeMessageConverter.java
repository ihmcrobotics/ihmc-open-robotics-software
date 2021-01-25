package us.ihmc.robotEnvironmentAwareness.communication.converters;

import controller_msgs.msg.dds.OcTreeKeyListMessage;
import net.jpountz.lz4.LZ4Exception;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeNodeMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.OcTreeKeyMessage;
import us.ihmc.tools.compression.LZ4CompressionImplementation;

import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.List;

public class OcTreeMessageConverter
{
   private static final ThreadLocal<LZ4CompressionImplementation> compressorThreadLocal = ThreadLocal.withInitial(LZ4CompressionImplementation::new);

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

   public static OcTreeKeyListMessage createOcTreeDataMessage(NormalOcTree normalOcTree)
   {
      OcTreeKeyListMessage ocTreeDataMessage = new OcTreeKeyListMessage();
      ocTreeDataMessage.setTreeResolution(normalOcTree.getResolution());
      ocTreeDataMessage.setTreeDepth(normalOcTree.getTreeDepth());

      List<OcTreeKey> ocTreeKeyList = new ArrayList<>();
      for (NormalOcTreeNode leaf : normalOcTree)
      {
         if (leaf.getDepth() == ocTreeDataMessage.getTreeDepth())
         {
            ocTreeKeyList.add(leaf.getKeyCopy());
         }
      }

      // three int's per key, each int is 4 bytes
      int ocTreeBufferSize = ocTreeKeyList.size() * 3 * 4;
      ByteBuffer rawOcTreeByteBuffer = ByteBuffer.allocate(ocTreeBufferSize);
      IntBuffer ocTreeIntBuffer = rawOcTreeByteBuffer.asIntBuffer();

      for (int i = 0; i < ocTreeKeyList.size(); i++)
      {
         ocTreeIntBuffer.put(3 * i, ocTreeKeyList.get(i).getKey(0));
         ocTreeIntBuffer.put(3 * i + 1, ocTreeKeyList.get(i).getKey(1));
         ocTreeIntBuffer.put(3 * i + 2, ocTreeKeyList.get(i).getKey(2));
      }

      LZ4CompressionImplementation compressor = compressorThreadLocal.get();
      ByteBuffer compressedOcTreeByteBuffer = ByteBuffer.allocate(ocTreeBufferSize);

      int compressedOcTreeSize;
      try
      {
         compressedOcTreeSize = compressor.compress(rawOcTreeByteBuffer, compressedOcTreeByteBuffer);
      }
      catch (LZ4Exception e)
      {
         e.printStackTrace();
         return null;
      }

      compressedOcTreeByteBuffer.flip();
      for (int i = 0; i < compressedOcTreeSize; i++)
      {
         ocTreeDataMessage.getKeys().add(compressedOcTreeByteBuffer.get());
      }

      ocTreeDataMessage.setNumberOfKeys(compressedOcTreeSize);
      return ocTreeDataMessage;
   }
}
