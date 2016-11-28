package us.ihmc.avatar.networkProcessor.quadTreeHeightMap;

import java.util.ArrayList;

import us.ihmc.humanoidRobotics.communication.packets.heightQuadTree.HeightQuadTreeMessage;
import us.ihmc.humanoidRobotics.communication.packets.heightQuadTree.HeightQuadTreeNodeMessage;
import us.ihmc.robotics.quadTree.Box;
import us.ihmc.robotics.quadTree.QuadTreeForGround;
import us.ihmc.robotics.quadTree.QuadTreeForGroundNode;

public class HeightQuadTreeMessageConverter
{
   public static HeightQuadTreeMessage convertQuadTreeForGround(QuadTreeForGround quadTreeToConvert)
   {
      QuadTreeForGroundNode rootNode = quadTreeToConvert.getRootNode();
      HeightQuadTreeNodeMessage rootNodeMessage = new HeightQuadTreeNodeMessage();

      fullDepthCopy(rootNode, rootNodeMessage);
      
      HeightQuadTreeMessage heightQuadTreeMessage = new HeightQuadTreeMessage();
      heightQuadTreeMessage.root = rootNodeMessage;
      heightQuadTreeMessage.defaultHeight = (float) rootNode.getDefaultHeightWhenNoPoints();
      heightQuadTreeMessage.resolution = (float) quadTreeToConvert.getQuadTreeParameters().getResolution();
      return heightQuadTreeMessage;
   }


   private static void fullDepthCopy(QuadTreeForGroundNode original, HeightQuadTreeNodeMessage copyToPack)
   {
      boolean isLeaf = original.getLeaf() != null;
      copyToPack.isLeaf = isLeaf;

      if(isLeaf)
         copyToPack.height = (float) original.getLeaf().getAveragePoint().getZ();

      Box bounds = original.getBounds();
      copyToPack.centerX = (float) bounds.centreX;
      copyToPack.centerY = (float) bounds.centreY;
      copyToPack.sizeX = (float) (bounds.maxX - bounds.minX);
      copyToPack.sizeY = (float) (bounds.maxY - bounds.minY);

      // Last depth level of the tree, we're done.
      if (isLeaf)
         return;

      if (original.hasChildren() && isAncestorOfAtLeastOneLeaf(original))
      {
         copyToPack.children = new HeightQuadTreeNodeMessage[4];
         ArrayList<QuadTreeForGroundNode> children = new ArrayList<>();
         original.getChildrenNodes(children);

         for (QuadTreeForGroundNode originalChild : children)
         {
            if (originalChild == null)
               continue;
            if (originalChild.isEmpty())
               continue;
            if (!isAncestorOfAtLeastOneLeaf(originalChild))
               continue;

            // Computing the morton code to make sure that the indexing is correct.
            int mortonCode = 0;
            if (originalChild.getBounds().centreX > copyToPack.centerX)
               mortonCode |= 1;
            if (originalChild.getBounds().centreY > copyToPack.centerY)
               mortonCode |= 2;

            HeightQuadTreeNodeMessage childCopy = new HeightQuadTreeNodeMessage();
            copyToPack.children[mortonCode] = childCopy;
            fullDepthCopy(originalChild, childCopy);
         }
      }
   }

   /**
    * It seems that {@link QuadTreeForGroundNode} can be created without actually holding any information.
    * In which case, no leaves can be found as going down the tree from such a node.
    * @param node this is the node to be tested to see if it is actually useful.
    * @return true if the node is the ancestor of at least one leaf (== useful), false otherwise (==useless).
    */
   private static boolean isAncestorOfAtLeastOneLeaf(QuadTreeForGroundNode node)
   {
      if (node.getLeaf() != null)
         return true;

      if (node.isEmpty())
         return false;

      ArrayList<QuadTreeForGroundNode> children = new ArrayList<>();
      node.getChildrenNodes(children);

      for (QuadTreeForGroundNode child : children)
      {
         if (isAncestorOfAtLeastOneLeaf(child))
            return true;
      }

      return false;
   }
}
