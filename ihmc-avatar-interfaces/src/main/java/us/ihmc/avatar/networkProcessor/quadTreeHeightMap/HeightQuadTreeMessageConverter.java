package us.ihmc.avatar.networkProcessor.quadTreeHeightMap;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.HeightQuadTreeLeafMessage;
import controller_msgs.msg.dds.HeightQuadTreeMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple2D.Point2D;
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
      MessageTools.copyData(leaves, heightQuadTreeMessage.getLeaves());
      heightQuadTreeMessage.setDefaultHeight((float) rootNode.getDefaultHeightWhenNoPoints());
      heightQuadTreeMessage.setResolution((float) quadTreeToConvert.getQuadTreeParameters().getResolution());
      Box bounds = quadTreeToConvert.getRootNode().getBounds();
      heightQuadTreeMessage.setSizeX((float) (bounds.maxX - bounds.minX));
      heightQuadTreeMessage.setSizeY((float) (bounds.maxY - bounds.minY));
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
         leaf.setCenterX((float) bounds.centreX);
         leaf.setCenterY((float) bounds.centreY);
         leaf.setHeight((float) original.getLeaf().getAveragePoint().getZ());
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
      heightQuadTree.setDefaultHeight(messageToConvert.getDefaultHeight());
      heightQuadTree.setResolution(messageToConvert.getResolution());
      heightQuadTree.setSizeX(messageToConvert.getSizeX());
      heightQuadTree.setSizeY(messageToConvert.getSizeY());

      if (messageToConvert.getLeaves().isEmpty())
         return heightQuadTree;

      HeightQuadTreeNode root = new HeightQuadTreeNode();
      root.setCenterX(0.0f);
      root.setCenterY(0.0f);
      root.setSizeX(messageToConvert.getSizeX());
      root.setSizeY(messageToConvert.getSizeY());

      for (int i = 0; i < messageToConvert.getLeaves().size(); i++)
         insertLeafRecursive(root, messageToConvert.getLeaves().get(i));

      heightQuadTree.setRoot(root);

      return heightQuadTree;
   }

   private static void insertLeafRecursive(HeightQuadTreeNode node, HeightQuadTreeLeafMessage leaf)
   {
      double epsilon = 1.0e-3;
      if (MathTools.epsilonEquals(node.getCenterX(), leaf.getCenterX(), epsilon) && MathTools.epsilonEquals(node.getCenterY(), leaf.getCenterY(), epsilon))
      {
         node.setHeight(leaf.getHeight());
         return;
      }

      if (!node.hasChildrenArray())
         node.assignChildrenArray();

      // Computing the morton code to make sure that the indexing is correct.
      int mortonCode = 0;
      if (leaf.getCenterX() > node.getCenterX())
         mortonCode |= 1;
      if (leaf.getCenterY() > node.getCenterY())
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
