package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.Arrays;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.tuple2D.Point2D;

/**
 * <p>
 * Title:
 * </p>
 * BoundingBoxKDTree2D
 * <p>
 * Description:
 * </p>
 * This class searches for bounding boxes present in a user-defined rectangular
 * region by constructing a KDTree for bounding boxes and then applying range
 * search to the KDTree.
 *
 * <p>
 * Copyright: Copyright (c) 2006
 * </p>
 *
 * <p>
 * Company:
 * </p>
 *
 * @author not attributable
 * @version 1.0
 */

public class BoundingBoxKDTree2D
{
   private KDTreeNode rootKDTreeNode;
   private ArrayList<BoundingBox2D> allboundingBoxes;
   @SuppressWarnings("unused")
   private int uniqueIdForDisplay = 0;
   private final ArrayList<Object> allObjects;
   private final int MAX_LEAF_SIZE = 5;

   /**
    * Constructor Builds KDTree with the bounding boxes Bounding Boxes must be
    * 2 dimensional rectangles oriented along X and Y axes.
    *
    * @param ArrayList
    *            <BoundingBox2D> boundingBoxes The list of the polygon to
    *            search from.
    */

   public BoundingBoxKDTree2D(ArrayList<BoundingBox2D> boundingBoxes, ArrayList<Object> objects)
   {
      rootKDTreeNode = new KDTreeNode(null);
      allboundingBoxes = boundingBoxes;
      allObjects = objects;

      if (allboundingBoxes.size() != allObjects.size())
      {
         System.err.println("KDTree::KDTree(): number of points and objects differ.");
      }

      buildKDTree(boundingBoxes, 0, rootKDTreeNode, objects);
   }

   @SuppressWarnings("unchecked")
   private KDTreeNode buildKDTree(ArrayList<BoundingBox2D> boundingBoxes, int depth, KDTreeNode nodeToExpand, ArrayList<Object> objects)
   {
      ArrayList<BoundingBox2D>[] split = new ArrayList[2];
      ArrayList<Object>[] objectSplit = new ArrayList[2];

      if (boundingBoxes.size() <= MAX_LEAF_SIZE)
      {
         nodeToExpand.boundingBoxes = boundingBoxes;
         nodeToExpand.objects = objects;
         nodeToExpand.leafNode = true;
      }
      else
      {
         if (depth % 2 == 0)    // even
         {
            // Split boxes vertically
            nodeToExpand.splitHorizontal = false;
            split = splitInXPlane(boundingBoxes, nodeToExpand, objects, objectSplit);
         }
         else
         {
            // split boxes horizontally
            nodeToExpand.splitHorizontal = true;
            split = splitInYPlane(boundingBoxes, nodeToExpand, objects, objectSplit);
         }

         if ((split[0].size() == boundingBoxes.size()) || (split[1].size() == boundingBoxes.size()))
         {
            nodeToExpand.leafNode = true;
            nodeToExpand.boundingBoxes = boundingBoxes;
            nodeToExpand.objects = objects;

            return nodeToExpand;

         }
         else
         {
            nodeToExpand.left = buildKDTree(split[0], depth + 1, new KDTreeNode(nodeToExpand), objectSplit[0]);

            if (!nodeToExpand.leafNode)
            {
               nodeToExpand.right = buildKDTree(split[1], depth + 1, new KDTreeNode(nodeToExpand), objectSplit[1]);

            }
         }
      }

      return nodeToExpand;
   }

   /**
    * Searches for bounding boxes present in/intersecting with a user-defined
    * rectangular search region. Search region must be a 2 dimensional
    * rectangle oriented along X and Y axes.
    *
    * @param BoundingBox2D
    *            searchBox The rectangular search region.
    * @return ArrayList<BoundingBox2D> The bounding boxes present
    *         in/intersecting with the search region.
    */

   public ArrayList<BoundingBox2D> getIntersectingBoundingBoxes(BoundingBox2D searchBox)
   {
      ArrayList<BoundingBox2D> intersectingBoundingBoxes = new ArrayList<BoundingBox2D>();
      ArrayList<Object> intersectingObjects = new ArrayList<Object>();
      getIntersectingBoundingBoxes(searchBox, rootKDTreeNode, 0, intersectingBoundingBoxes, intersectingObjects);

      return intersectingBoundingBoxes;
   }

   public ArrayList<Object> getIntersectingObjects(BoundingBox2D searchBox)
   {
      ArrayList<BoundingBox2D> intersectingBoundingBoxes = new ArrayList<BoundingBox2D>();
      ArrayList<Object> intersectingObjects = new ArrayList<Object>();
      getIntersectingBoundingBoxes(searchBox, rootKDTreeNode, 0, intersectingBoundingBoxes, intersectingObjects);

      return intersectingObjects;
   }


   private void getIntersectingBoundingBoxes(BoundingBox2D searchBox, KDTreeNode current, int depth, ArrayList<BoundingBox2D> intersectingBoundingBoxes,
           ArrayList<Object> intersectingObjects)
   {
      if (current.leafNode)
      {
         addToReturnList(current.boundingBoxes, current.objects, searchBox, intersectingBoundingBoxes, intersectingObjects);
      }
      else if (depth % 2 == 0)    // even
      {
         // see if point is left or right
         if (searchBox.getMaxX() <= current.split)    // left
            getIntersectingBoundingBoxes(searchBox, current.left, depth + 1, intersectingBoundingBoxes, intersectingObjects);
         else if (searchBox.getMinX() >= current.split)
            getIntersectingBoundingBoxes(searchBox, current.right, depth + 1, intersectingBoundingBoxes, intersectingObjects);
         else
         {
            // search both directions
            getIntersectingBoundingBoxes(searchBox, current.left, depth + 1, intersectingBoundingBoxes, intersectingObjects);
            getIntersectingBoundingBoxes(searchBox, current.right, depth + 1, intersectingBoundingBoxes, intersectingObjects);
         }

      }
      else if (searchBox.getMinY() >= current.split)    // above
         getIntersectingBoundingBoxes(searchBox, current.left, depth + 1, intersectingBoundingBoxes, intersectingObjects);
      else if (searchBox.getMaxY() <= current.split)
         getIntersectingBoundingBoxes(searchBox, current.right, depth + 1, intersectingBoundingBoxes, intersectingObjects);
      else
      {
         // search both directions
         getIntersectingBoundingBoxes(searchBox, current.left, depth + 1, intersectingBoundingBoxes, intersectingObjects);
         getIntersectingBoundingBoxes(searchBox, current.right, depth + 1, intersectingBoundingBoxes, intersectingObjects);
      }
   }

   private void addToReturnList(ArrayList<BoundingBox2D> currentBoxes, ArrayList<Object> currentObjects, BoundingBox2D searchBox,
                                ArrayList<BoundingBox2D> intersectingBoundingBoxes, ArrayList<Object> intersectingObjects)
   {
      for (int i = 0; i < currentBoxes.size(); i++)

//    for (BoundingBox2D b : currentBoxes)
      {
         if (!intersectingBoundingBoxes.contains(currentBoxes.get(i)))
         {
            if (searchBox.intersectsInclusive(currentBoxes.get(i)))
            {
               intersectingBoundingBoxes.add(currentBoxes.get(i));
               intersectingObjects.add(currentObjects.get(i));
            }
         }
      }
   }

   @SuppressWarnings("unchecked")
   private ArrayList<BoundingBox2D>[] splitInXPlane(ArrayList<BoundingBox2D> boundingBoxes, KDTreeNode current, ArrayList<Object> objects,
           ArrayList<Object>[] objectSplit)
   {
      ArrayList<BoundingBox2D> leftList = new ArrayList<BoundingBox2D>();
      ArrayList<BoundingBox2D> rightList = new ArrayList<BoundingBox2D>();
      ArrayList<Object> leftObjectList = new ArrayList<Object>();
      ArrayList<Object> rightObjectList = new ArrayList<Object>();

      ArrayList<BoundingBox2D>[] returnList = new ArrayList[2];

      Point2D center = new Point2D();
      Point2D minPoint = new Point2D();
      Point2D maxPoint = new Point2D();

      double[] midPointsX = new double[boundingBoxes.size()];
      for (int i = 0; i < boundingBoxes.size(); i++)
      {
         boundingBoxes.get(i).getCenterPoint(center);
         midPointsX[i] = center.getX();
      }

      double median = getMedian(midPointsX.clone());
      current.split = median;

      for (int i = 0; i < boundingBoxes.size(); i++)
      {
         boundingBoxes.get(i).getMinPoint(minPoint);
         boundingBoxes.get(i).getMaxPoint(maxPoint);

         if (median <= minPoint.getX())
         {
            rightList.add(boundingBoxes.get(i));
            rightObjectList.add(objects.get(i));
         }
         else if (median >= maxPoint.getX())
         {
            leftList.add(boundingBoxes.get(i));
            leftObjectList.add(objects.get(i));
         }
         else
         {
            rightList.add(boundingBoxes.get(i));
            rightObjectList.add(objects.get(i));
            leftList.add(boundingBoxes.get(i));
            leftObjectList.add(objects.get(i));
         }

      }

      returnList[0] = leftList;
      returnList[1] = rightList;
      objectSplit[0] = leftObjectList;
      objectSplit[1] = rightObjectList;

      return returnList;
   }

   @SuppressWarnings("unchecked")
   private ArrayList<BoundingBox2D>[] splitInYPlane(ArrayList<BoundingBox2D> boundingBoxes, KDTreeNode current, ArrayList<Object> objects,
           ArrayList<Object>[] objectSplit)
   {
      ArrayList<BoundingBox2D> upperList = new ArrayList<BoundingBox2D>();
      ArrayList<BoundingBox2D> lowerList = new ArrayList<BoundingBox2D>();
      ArrayList<Object> upperObjectList = new ArrayList<Object>();
      ArrayList<Object> lowerObjectList = new ArrayList<Object>();

      ArrayList<BoundingBox2D>[] returnList = new ArrayList[2];

      Point2D center = new Point2D();
      Point2D minPoint = new Point2D();
      Point2D maxPoint = new Point2D();

      double[] midPointsY = new double[boundingBoxes.size()];
      for (int i = 0; i < boundingBoxes.size(); i++)
      {
         boundingBoxes.get(i).getCenterPoint(center);
         midPointsY[i] = center.getY();
      }

      double median = getMedian(midPointsY.clone());
      current.split = median;

      for (int i = 0; i < boundingBoxes.size(); i++)
      {
         boundingBoxes.get(i).getMinPoint(minPoint);
         boundingBoxes.get(i).getMaxPoint(maxPoint);

         if (median <= minPoint.getY())
         {
            upperList.add(boundingBoxes.get(i));
            upperObjectList.add(objects.get(i));
         }
         else if (median >= maxPoint.getY())
         {
            lowerList.add(boundingBoxes.get(i));
            lowerObjectList.add(objects.get(i));
         }
         else
         {
            upperList.add(boundingBoxes.get(i));
            upperObjectList.add(objects.get(i));
            lowerList.add(boundingBoxes.get(i));
            lowerObjectList.add(objects.get(i));
         }

      }

      returnList[0] = upperList;
      returnList[1] = lowerList;
      objectSplit[0] = upperObjectList;
      objectSplit[1] = lowerObjectList;

      return returnList;
   }

   private double getMedian(double[] list)
   {
      Arrays.sort(list);
      int middle = list.length / 2;
      if (list.length % 2 == 1)
      {
         return list[middle];
      }
      else
      {
         return (list[middle - 1] + list[middle]) / 2.0;
      }

   }

   private class KDTreeNode
   {
      private double split;
      @SuppressWarnings("unused")
      private boolean splitHorizontal;
      private KDTreeNode left = null;
      private KDTreeNode right = null;
      @SuppressWarnings("unused")
      private KDTreeNode parent = null;
      private boolean leafNode = false;
      private ArrayList<BoundingBox2D> boundingBoxes;
      private ArrayList<Object> objects;

      private KDTreeNode(KDTreeNode parent)
      {
         this.parent = parent;
      }
   }

}
