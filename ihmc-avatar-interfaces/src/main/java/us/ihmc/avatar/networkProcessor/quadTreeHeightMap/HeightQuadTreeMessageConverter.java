package us.ihmc.avatar.networkProcessor.quadTreeHeightMap;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.communication.packets.heightQuadTree.HeightQuadTreeLeafMessage;
import us.ihmc.humanoidRobotics.communication.packets.heightQuadTree.HeightQuadTreeMessage;
import us.ihmc.robotics.quadTree.Box;
import us.ihmc.robotics.quadTree.QuadTreeForGround;
import us.ihmc.robotics.quadTree.QuadTreeForGroundNode;

public class HeightQuadTreeMessageConverter
{
   public static HeightQuadTreeMessage convertQuadTreeForGround(QuadTreeForGround quadTreeToConvert)
   {
      return convertQuadTreeForGround(quadTreeToConvert, null, Double.POSITIVE_INFINITY);
   }

   public static HeightQuadTreeMessage convertQuadTreeForGround(QuadTreeForGround quadTreeToConvert, Point2D boundingCircleCenter, double boundingCircleRadius)
   {
      QuadTreeForGroundNode rootNode = quadTreeToConvert.getRootNode();

      List<HeightQuadTreeLeafMessage> leaves = new ArrayList<>();
      fullDepthCopy(rootNode, boundingCircleCenter, boundingCircleRadius, leaves);

      HeightQuadTreeMessage heightQuadTreeMessage = new HeightQuadTreeMessage();
      MessageTools.copyData(leaves, heightQuadTreeMessage.leaves);
      heightQuadTreeMessage.defaultHeight = (float) rootNode.getDefaultHeightWhenNoPoints();
      heightQuadTreeMessage.resolution = (float) quadTreeToConvert.getQuadTreeParameters().getResolution();
      Box bounds = quadTreeToConvert.getRootNode().getBounds();
      heightQuadTreeMessage.sizeX = (float) (bounds.maxX - bounds.minX);
      heightQuadTreeMessage.sizeY = (float) (bounds.maxY - bounds.minY);
      return heightQuadTreeMessage;
   }

   private static void fullDepthCopy(QuadTreeForGroundNode original, Point2D boundingCircleCenter, double boundingCircleRadius,
                                     List<HeightQuadTreeLeafMessage> copyToPack)
   {
      boolean isLeaf = original.getLeaf() != null;

      if (isLeaf)
      {
         HeightQuadTreeLeafMessage leaf = new HeightQuadTreeLeafMessage();
         Box bounds = original.getBounds();
         leaf.center.set(bounds.centreX, bounds.centreY);
         leaf.height = (float) original.getLeaf().getAveragePoint().getZ();
         copyToPack.add(leaf);
         return;
      }

      if (original.hasChildren() && isAncestorOfAtLeastOneLeafInsideBoundingCircle(original, boundingCircleCenter, boundingCircleRadius))
      {
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

            fullDepthCopy(originalChild, boundingCircleCenter, boundingCircleRadius, copyToPack);
         }
      }
   }

   private static boolean isAncestorOfAtLeastOneLeafInsideBoundingCircle(QuadTreeForGroundNode node, Point2D boundingCircleCenter, double boundingCircleRadius)
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

   private static boolean isInsideBoundingCircle(double x, double y, Point2D boundingCircleCenter, double boundingCircleRadius)
   {
      double dx = x - boundingCircleCenter.getX();
      double dy = y - boundingCircleCenter.getY();
      return dx * dx + dy * dy < boundingCircleRadius * boundingCircleRadius;
   }

   public static HeightQuadTree convertMessage(HeightQuadTreeMessage messageToConvert)
   {
      HeightQuadTree heightQuadTree = new HeightQuadTree();
      heightQuadTree.setDefaultHeight(messageToConvert.defaultHeight);
      heightQuadTree.setResolution(messageToConvert.resolution);
      heightQuadTree.setSizeX(messageToConvert.sizeX);
      heightQuadTree.setSizeY(messageToConvert.sizeY);

      if (messageToConvert.leaves.isEmpty())
         return heightQuadTree;

      HeightQuadTreeNode root = new HeightQuadTreeNode();
      root.setCenterX(0.0f);
      root.setCenterY(0.0f);
      root.setSizeX(messageToConvert.sizeX);
      root.setSizeY(messageToConvert.sizeY);

      for (int i = 0; i < messageToConvert.leaves.size(); i++)
         insertLeafRecursive(root, messageToConvert.leaves.get(i));

      heightQuadTree.setRoot(root);

      return heightQuadTree;
   }

   private static void insertLeafRecursive(HeightQuadTreeNode node, HeightQuadTreeLeafMessage leaf)
   {
      if (leaf.center.epsilonEquals(node.getCenter(), 1.0e-3))
      {
         node.setHeight(leaf.height);
         return;
      }

      if (!node.hasChildrenArray())
         node.assignChildrenArray();

      // Computing the morton code to make sure that the indexing is correct.
      int mortonCode = 0;
      if (leaf.center.getX32() > node.getCenterX())
         mortonCode |= 1;
      if (leaf.center.getY32() > node.getCenterY())
         mortonCode |= 2;

      HeightQuadTreeNode child = node.getChild(mortonCode);

      if (child == null)
      {
         child = new HeightQuadTreeNode();
         child.setSizeX(0.5f * node.getSizeX());
         child.setSizeY(0.5f * node.getSizeY());

         if ((mortonCode & 1) != 0)
            child.setCenterX(node.getCenterX() + 0.5f * child.getSizeX());
         else
            child.setCenterX(node.getCenterX() - 0.5f * child.getSizeX());

         if ((mortonCode & 2) != 0)
            child.setCenterY(node.getCenterY() + 0.5f * child.getSizeY());
         else
            child.setCenterY(node.getCenterY() - 0.5f * child.getSizeY());

         node.setChild(mortonCode, child);
      }

      insertLeafRecursive(child, leaf);
   }
}
