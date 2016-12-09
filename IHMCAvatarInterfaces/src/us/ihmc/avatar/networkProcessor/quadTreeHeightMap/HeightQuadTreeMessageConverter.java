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
      Box bounds = quadTreeToConvert.getRootNode().getBounds();
      heightQuadTreeMessage.sizeX = (float) (bounds.maxX - bounds.minX);
      heightQuadTreeMessage.sizeY = (float) (bounds.maxY - bounds.minY);
      return heightQuadTreeMessage;
   }


   private static void fullDepthCopy(QuadTreeForGroundNode original, Point2d boundingCircleCenter, double boundingCircleRadius, HeightQuadTreeNodeMessage copyToPack)
   {
      boolean isLeaf = original.getLeaf() != null;

      if(isLeaf)
      {
         copyToPack.height = (float) original.getLeaf().getAveragePoint().getZ();
         return;
      }

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
            if (originalChild.getBounds().centreX > original.getBounds().centreX)
               mortonCode |= 1;
            if (originalChild.getBounds().centreY > original.getBounds().centreY)
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

   public static HeightQuadTree convertMessage(HeightQuadTreeMessage messageToConvert)
   {
      HeightQuadTree heightQuadTree = new HeightQuadTree();
      heightQuadTree.setDefaultHeight(messageToConvert.defaultHeight);
      heightQuadTree.setResolution(messageToConvert.resolution);
      heightQuadTree.setSizeX(messageToConvert.sizeX);
      heightQuadTree.setSizeY(messageToConvert.sizeY);

      if (messageToConvert.root == null)
         return heightQuadTree;

      HeightQuadTreeNode root = new HeightQuadTreeNode();
      root.setHeight(messageToConvert.root.height);
      root.setCenterX(0.0f);
      root.setCenterY(0.0f);
      root.setSizeX(messageToConvert.sizeX);
      root.setSizeY(messageToConvert.sizeY);

      fullDepthCopy(root, messageToConvert.root);
      heightQuadTree.setRoot(root);

      return heightQuadTree;
   }

   private static void fullDepthCopy(HeightQuadTreeNode node, HeightQuadTreeNodeMessage nodeMessage)
   {
      if (nodeMessage.children == null)
         return;

      node.assignChildrenArray();

      for (int childIndex = 0; childIndex < 4; childIndex++)
      {
         HeightQuadTreeNodeMessage childMessage = nodeMessage.children[childIndex];
         if (childMessage == null)
            continue;

         HeightQuadTreeNode child = new HeightQuadTreeNode();
         child.setSizeX(0.5f * node.getSizeX());
         child.setSizeY(0.5f * node.getSizeY());

         if ((childIndex & 1) != 0)
            child.setCenterX(node.getCenterX() + 0.5f * child.getSizeX());
         else
            child.setCenterX(node.getCenterX() - 0.5f * child.getSizeX());

         if ((childIndex & 2) != 0)
            child.setCenterY(node.getCenterY() + 0.5f * child.getSizeY());
         else
            child.setCenterY(node.getCenterY() - 0.5f * child.getSizeY());

         child.setHeight(childMessage.height);
         node.setChild(childIndex, child);
         fullDepthCopy(child, childMessage);
      }
   }
}
