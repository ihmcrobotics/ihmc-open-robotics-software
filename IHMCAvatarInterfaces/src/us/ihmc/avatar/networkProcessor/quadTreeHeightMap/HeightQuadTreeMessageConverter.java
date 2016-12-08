package us.ihmc.avatar.networkProcessor.quadTreeHeightMap;

import java.util.ArrayList;

import javax.vecmath.Point2d;

import us.ihmc.humanoidRobotics.communication.packets.heightQuadTree.HeightQuadTreeMessage;
import us.ihmc.humanoidRobotics.communication.packets.heightQuadTree.HeightQuadTreeNodeMessage;
import us.ihmc.robotics.quadTree.Box;
import us.ihmc.robotics.quadTree.QuadTreeForGround;
import us.ihmc.robotics.quadTree.QuadTreeForGroundNode;

public class HeightQuadTreeMessageConverter
{
   public static HeightQuadTreeMessage convertQuadTreeForGround(QuadTreeForGround quadTreeToConvert)
   {
      return convertQuadTreeForGround(quadTreeToConvert, null, Double.POSITIVE_INFINITY);
   }

   public static HeightQuadTreeMessage convertQuadTreeForGround(QuadTreeForGround quadTreeToConvert, Point2d boundingCircleCenter, double boundingCircleRadius)
   {
      QuadTreeForGroundNode rootNode = quadTreeToConvert.getRootNode();
      HeightQuadTreeNodeMessage rootNodeMessage = new HeightQuadTreeNodeMessage();

      fullDepthCopy(rootNode, boundingCircleCenter, boundingCircleRadius, rootNodeMessage);
      
      HeightQuadTreeMessage heightQuadTreeMessage = new HeightQuadTreeMessage();
      heightQuadTreeMessage.root = rootNodeMessage;
      heightQuadTreeMessage.defaultHeight = (float) rootNode.getDefaultHeightWhenNoPoints();
      heightQuadTreeMessage.resolution = (float) quadTreeToConvert.getQuadTreeParameters().getResolution();
      return heightQuadTreeMessage;
   }


   private static void fullDepthCopy(QuadTreeForGroundNode original, Point2d boundingCircleCenter, double boundingCircleRadius, HeightQuadTreeNodeMessage copyToPack)
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

      if (original.hasChildren() && isAncestorOfAtLeastOneLeafInsideBoundingCircle(original, boundingCircleCenter, boundingCircleRadius))
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
            if (!isAncestorOfAtLeastOneLeafInsideBoundingCircle(originalChild, boundingCircleCenter, boundingCircleRadius))
               continue;

            // Computing the morton code to make sure that the indexing is correct.
            int mortonCode = 0;
            if (originalChild.getBounds().centreX > copyToPack.centerX)
               mortonCode |= 1;
            if (originalChild.getBounds().centreY > copyToPack.centerY)
               mortonCode |= 2;

            HeightQuadTreeNodeMessage childCopy = new HeightQuadTreeNodeMessage();
            copyToPack.children[mortonCode] = childCopy;
            fullDepthCopy(originalChild, boundingCircleCenter, boundingCircleRadius, childCopy);
         }
      }
   }

   private static boolean isAncestorOfAtLeastOneLeafInsideBoundingCircle(QuadTreeForGroundNode node, Point2d boundingCircleCenter, double boundingCircleRadius)
   {
      if (node.getLeaf() != null)
         return isInsideBoundingCircle(node.getBounds().centreX, node.getBounds().centreY, boundingCircleCenter, boundingCircleRadius);

      if (node.isEmpty())
         return false;

      ArrayList<QuadTreeForGroundNode> children = new ArrayList<>();
      node.getChildrenNodes(children);

      for (QuadTreeForGroundNode child : children)
      {
         if (isAncestorOfAtLeastOneLeafInsideBoundingCircle(child, boundingCircleCenter, boundingCircleRadius))
            return true;
      }

      return false;
   }

   private static boolean isInsideBoundingCircle(double x, double y, Point2d boundingCircleCenter, double boundingCircleRadius)
   {
      double dx = x - boundingCircleCenter.getX();
      double dy = y - boundingCircleCenter.getY();
      return dx * dx + dy * dy < boundingCircleRadius * boundingCircleRadius;
   }
}
