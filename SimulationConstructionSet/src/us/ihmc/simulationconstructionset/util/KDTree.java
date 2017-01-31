package us.ihmc.simulationconstructionset.util;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.Random;
import java.util.StringTokenizer;

import javax.vecmath.Point3d;

/**
 * Use this class to efficiently search for closest points to various test
 * points of interest.  The dimensionality of the points is arbitrary (D > 0), and
 * the tree's splitting is binary (K = 2).
 */
public class KDTree
{
   // Allow some member data changes in case of point pruning.
   private final boolean hasObjects;
   private final int maxPointsInLeaves;
   private final Object[] objects;
   private final KDNode root;
   private LinkedHashSet<Integer> novelPointIndexes;
   private static final boolean DEBUG = false;

   /**
    * Creates a KDTree from an array of points and an equally sized array of
    * objects.  The value MaxPointsInLeaves specifies the maximum number of
    * points in a leaf Node.   Use a small value (5-20) unless building the
    * tree takes too long.
    *
    * @param points double[][]
    * @param objects Object[]
    * @param maxPointsInLeaves int
    */
   public KDTree(double[][] points, Object[] objects, int maxPointsInLeaves)
   {
      if (points == objects)
      {
         throw new RuntimeException("Do not use the same keys and values in constructing a KDTree. If they are the same, then just use the key constructor");
      }

      makeAllPointsNovel(points.length);

      this.hasObjects = true;
      this.objects = objects;
      this.maxPointsInLeaves = maxPointsInLeaves;

      if (points.length != objects.length)
      {
         System.err.println("KDTree::KDTree(): number of points and objects differ.  Creating empty KDTree.");
         this.root = null;
      }
      else
      {
         this.root = new KDNode(points, 0, points.length - 1, maxPointsInLeaves);
      }
   }

   /**
    * Creates a KDTree from an array of points and an equally sized array of
    * objects.  The value MaxPointsInLeaves specifies the maximum number of
    * points in a leaf Node.   Use a small value (5-20) unless building the
    * tree takes too long.
    *
    * @param points double[][]
    * @param maxPointsInLeaves int
    */
   public KDTree(double[][] points, int maxPointsInLeaves)
   {
      this.hasObjects = false;
      this.objects = null;
      this.maxPointsInLeaves = maxPointsInLeaves;
      makeAllPointsNovel(points.length);
      this.root = new KDNode(points, 0, points.length - 1, maxPointsInLeaves);
   }


   /**
    * Creates a KDTree from an array of (X, Y) terrain points and an equally sized array of
    * (Z) terrain heights.  The value MaxPointsInLeaves specifies the maximum number of
    * points in a leaf Node.   Use a small value (5-20) unless building the
    * tree takes too long.
    *
    * @param points double[][]
    * @param maxPointsInLeaves int
    */
   public KDTree(String BDITerrainFilePath, int maxPointsInLeaves)
   {
      this.hasObjects = false;
      this.objects = null;
      this.maxPointsInLeaves = maxPointsInLeaves;
      double[][] points = loadPoints3D(BDITerrainFilePath);
      makeAllPointsNovel(points.length);
      this.root = new KDNode(points, 0, points.length - 1, maxPointsInLeaves);
   }

   private void makeAllPointsNovel(int numPoints)
   {
      novelPointIndexes = new LinkedHashSet<Integer>();

      for (int i = 0; i < numPoints; i++)
      {
         novelPointIndexes.add(i);
      }
   }

   /**
    * Returns the list of points.  The user may not have this in advance if
    * the points are loaded from a file.
    *
    * @return double[][]
    */
   public double[][] getPoints()
   {
      return root.points;
   }

   public void getExtents(double[] min, double[] max)
   {
      root.getExtents(min, max);
   }

// // Delete points in the KDTree such that the remaining
// // points are no closer than minDistanceBetweenPoints from each other.
// // The deleted points cannot be restored once this call is made.
// public void prunePoints(double minDistanceBetweenPoints)
// {
//     int numPoints = root.points.length;
//     int numDimensions = root.points[0].length;
//
//     // Track which points are not in the covering set.
//     // Start with all points in the cover and remove
//     // points incrementally.
//     HashSet<Integer> inCover = new LinkedHashSet<Integer>();
//     for (int i=0; i<numPoints; i++)  inCover.add(i);
//
//     ArrayList<Integer> coverIndexes = new ArrayList<Integer>();
//
//     // Loop over all points.  For any point still in the
//     // cover, find all other points nearby and remove them
//     // from the cover.
//     for (int i=0; i<numPoints; i++)
//     {
//        if (inCover.contains(i))
//         {
//             coverIndexes.add(i);
//             ArrayList<Integer> coveredIndexes = closestPointIndexesInSubset(root.points[i], minDistanceBetweenPoints, inCover);
//             inCover.removeAll(coveredIndexes);
//         }
//     }
//
//     // Collect points in the cover and rebuild the tree with these points only.
//     // If Objects are associated with points, we collect the objects also.
//     int numPointsInCover = coverIndexes.size();
//     double[][] newPoints = new double[numPointsInCover][numDimensions];
//     if (!hasObjects)
//     {
//         for (int i=0; i<numPointsInCover; i++)
//         {
//             newPoints[i] = root.points[coverIndexes.get(i)];
//         }
//         this.root = new KDNode(newPoints, 0, newPoints.length - 1, maxPointsInLeaves);
//     }
//     else
//     {
//         Object [] newObjects = new Object[numPointsInCover];
//         for (int i=0; i<numPointsInCover; i++)
//         {
//             newPoints[i] = root.points[coverIndexes.get(i)];
//             newObjects[i] = objects[coverIndexes.get(i)];
//         }
//         this.objects = newObjects;
//         this.root = new KDNode(newPoints, 0, newPoints.length - 1, maxPointsInLeaves);
//     }
// }
//

   // Delete points in the KDTree such that the remaining
   // points are no closer than minDistanceBetweenPoints from each other.
   // The deleted points cannot be restored once this call is made.
   public KDTree prunePointsCopy(double minDistanceBetweenPoints)
   {
      int numPoints = root.points.length;
      makeAllPointsNovel(numPoints);
      int numDimensions = root.points[0].length;

      // Track which points are not in the covering set.
      // Start with all points in the cover and remove
      // points incrementally.
      HashSet<Integer> inCover = novelPointIndexes;

      ArrayList<Integer> coverIndexes = new ArrayList<Integer>();

      // Loop over all points.  For any point still in the
      // cover, find all other points nearby and remove them
      // from the cover.
      for (int i = 0; i < numPoints; i++)
      {
         if (inCover.contains(i))
         {
            coverIndexes.add(i);
            @SuppressWarnings("unused")
            ArrayList<Integer> coveredIndexes = this.getClosestNovelPointIndexes(root.points[i], minDistanceBetweenPoints, false);
            if (DEBUG)
            {
               System.err.println("Found cover point; number of novel points remaining: " + inCover.size());
            }

            // inCover.removeAll(coveredIndexes);
         }
      }

      // Collect points in the cover and rebuild the tree with these points only.
      // If Objects are associated with points, we collect the objects also.
      int numPointsInCover = coverIndexes.size();
      double[][] newPoints = new double[numPointsInCover][numDimensions];
      if (!hasObjects)
      {
         for (int i = 0; i < numPointsInCover; i++)
         {
            newPoints[i] = root.points[coverIndexes.get(i)];
         }

//       this.root = new KDNode(newPoints, 0, newPoints.length - 1, maxPointsInLeaves);

         return new KDTree(newPoints, maxPointsInLeaves);
      }
      else
      {
         Object[] newObjects = new Object[numPointsInCover];
         for (int i = 0; i < numPointsInCover; i++)
         {
            newPoints[i] = root.points[coverIndexes.get(i)];
            newObjects[i] = objects[coverIndexes.get(i)];
         }

         return new KDTree(newPoints, newObjects, maxPointsInLeaves);

//       this.objects = newObjects;
//       this.root = new KDNode(newPoints, 0, newPoints.length - 1, maxPointsInLeaves);
      }
   }



   /**
    * closestPoint
    *
    * @param testPoint double[]
    * @return double[]
    */
   public double[] closestPoint(double[] testPoint)
   {
      return root.closestPoint(testPoint);
   }

   public double[] closestPoint(double[] testPoint, double maxDistance)
   {
      return root.closestPoint(testPoint, maxDistance * maxDistance);
   }


   public double[] closestPointBruteForceSearch(double[] testPoint)
   {
      return root.closestPointBruteForceSearch(testPoint);
   }

   public Object closestObject(double[] testPoint)
   {
      if (!hasObjects)
      {
         System.err.println("KDTree::closestObject(): no objects exist. Returning null.");

         return null;
      }

      int closestIndex = root.closestPointIndex(testPoint);

      return objects[closestIndex];
   }

   public Object closestObject(double[] testPoint, double maxDistance)
   {
      if (!hasObjects)
      {
         System.err.println("KDTree::closestObject(): no objects exist. Returning null.");

         return null;
      }

      int closestIndex = root.closestPointIndex(testPoint, maxDistance * maxDistance, null, null);
      if (closestIndex == -1)
         return null;

      return objects[closestIndex];
   }


   public ArrayList<Object> closestObjects(double[] testPoint, int numObjects, double maxDistance)
   {
      if (!hasObjects)
      {
         System.err.println("KDTree::closestObject(): no objects exist. Returning null.");

         return null;
      }

      ArrayList<Integer> excludedIndexes = new ArrayList<Integer>();
      ArrayList<Object> nearestObjects = new ArrayList<Object>();

      for (int i = 0; i < numObjects; i++)
      {
         int closestIndex = root.closestPointIndex(testPoint, maxDistance * maxDistance, excludedIndexes, null);
         if (closestIndex == -1)
            break;
         excludedIndexes.add(closestIndex);
         nearestObjects.add(objects[closestIndex]);
      }

      return nearestObjects;
   }



   public ArrayList<double[]> closestPoints(double[] testPoint, int numPoints, double maxDistance)
   {
      ArrayList<Integer> excludedIndexes = new ArrayList<Integer>();
      ArrayList<double[]> nearestPoints = new ArrayList<double[]>();

      for (int i = 0; i < numPoints; i++)
      {
         int closestIndex = root.closestPointIndex(testPoint, maxDistance * maxDistance, excludedIndexes, null);
         if (closestIndex == -1)
            break;
         excludedIndexes.add(closestIndex);
         nearestPoints.add(root.points[closestIndex]);
      }

      return nearestPoints;
   }


   public Object getClosestNovelObject(double[] testPoint, double maxDistance, boolean preserveAsNovel)
   {
      int closestNovelIndex = getClosestNovelPointIndex(testPoint, maxDistance, preserveAsNovel);

      return objects[closestNovelIndex];
   }

   // Returns the indexes for all points within a given radius that have
   // not been found since all points were marked as novel.
   public ArrayList<Object> getClosestNovelObjects(double[] testPoint, double maxDistance, boolean preserveAsNovel)
   {
      ArrayList<Integer> closestNovelIndexes = getClosestNovelPointIndexes(testPoint, maxDistance, preserveAsNovel);
      ArrayList<Object> closestObjects = new ArrayList<Object>();
      for (int index : closestNovelIndexes)
      {
         closestObjects.add(objects[index]);
      }

      return closestObjects;
   }


   private void makeIndexesNovel(ArrayList<Integer> indexes)
   {
      novelPointIndexes.addAll(indexes);
   }

   // Returns the indexes for all points within a given radius that have
   // not been found since all points were marked as novel.
   private ArrayList<Integer> getClosestNovelPointIndexes(double[] testPoint, double maxDistance, boolean preserveAsNovel)
   {
      ArrayList<Integer> nearestIndexes = new ArrayList<Integer>();
      int closestIndex = getClosestNovelPointIndex(testPoint, maxDistance, false);

      while (closestIndex >= 0)
      {
         nearestIndexes.add(closestIndex);
         closestIndex = getClosestNovelPointIndex(testPoint, maxDistance, false);
      }

      if (preserveAsNovel)
      {
         makeIndexesNovel(nearestIndexes);
      }

      return nearestIndexes;
   }

   private int getClosestNovelPointIndex(double[] testPoint, double maxDistance, boolean preserveAsNovel)
   {
      int closestIndex = root.closestPointIndex(testPoint, maxDistance * maxDistance, null, novelPointIndexes);
      if ((closestIndex >= 0) &&!preserveAsNovel)
      {
         novelPointIndexes.remove(closestIndex);
      }

      return closestIndex;
   }



   // Returns the indexes for all points within a given radius.
// private ArrayList<Integer> closestPointIndexes(double[] testPoint, double maxDistance, ArrayList<Integer> excludeIndexes)
// {
//     ArrayList<Integer> nearestIndexes =  new ArrayList<Integer>();
//
//     int closestIndex = root.closestPointIndex(testPoint, maxDistance*maxDistance, excludeIndexes, null);
//     while (closestIndex >= 0)
//     {
//         nearestIndexes.add(closestIndex);
//         excludeIndexes.add(closestIndex);
//         closestIndex = root.closestPointIndex(testPoint, maxDistance*maxDistance, excludeIndexes, null);
//     }
//     return nearestIndexes;
// }


   /**
    * Loads an ASCII file of 3D points.  The first line must contain the
    * number of points, and all subsequent lines must contain three scalar values.
    *
    * @param filename String
    * @return OneDTerrainGrid
    */
   public static double[][] loadPoints3D(String filename)
   {
      BufferedReader bufferedReader;
      try
      {
         bufferedReader = new BufferedReader(new FileReader(filename));
         System.out.println("Found " + filename);
         double[][] points = loadPoints3D(bufferedReader);
         bufferedReader.close();

         return points;
      }
      catch (IOException e)
      {
         System.out.println("Could not open " + filename);
      }

      return null;
   }


   /**
    * Loads terrain data from a BufferedReader and returns the Terrain object
    * represented by the data. Returns null if the operation does not succeed.
    * There must be 3 scalar values on each line.
    *
    * @param bufferedReader BufferedReader
    * @return BreadthFirstStateEnumerator
    */
   public static double[][] loadPoints3D(BufferedReader bufferedReader)
   {
      ArrayList<Point3d> pointArray = new ArrayList<Point3d>();
      try
      {
         String lineIn;
         do
         {
            lineIn = bufferedReader.readLine();

            if (lineIn != null)
            {
               StringTokenizer s = new StringTokenizer(lineIn, " ");
               double x = Double.parseDouble(s.nextToken());
               double y = Double.parseDouble(s.nextToken());
               double z = Double.parseDouble(s.nextToken());
               if (s.hasMoreTokens())
               {
                  System.err.println("KDTree::loadPoints3D(): extra element ");
               }

               pointArray.add(new Point3d(x, y, z));
            }
         }
         while (lineIn != null);

         double[][] points = new double[pointArray.size()][3];
         for (int i = 0; i < pointArray.size(); i++)
         {
            points[i] = new double[] {pointArray.get(i).getX(), pointArray.get(i).getY(), pointArray.get(i).getZ()};
         }

         return points;
      }

      catch (IOException ex)
      {
         System.err.println(ex);
      }
      catch (NumberFormatException ex)
      {
         ex.printStackTrace();

         // System.err.println(ex);
         System.exit(-1);
      }

      return null;
   }

   /**
    * This is a fast Euclidean distance metric.
    *
    * @param point1 double[]
    * @param point2 double[]
    * @return double
    */
   public static double distanceSquared(double[] point1, double[] point2)
   {
      if (point1.length != point2.length)
      {
         System.err.println("KDTree::Node::distanceSquared(): point dimensions do not match.  Returning NaN.");

         return Double.NaN;
      }

      double distanceSquared = 0.0;
      for (int i = 0; i < point1.length; i++)
      {
         distanceSquared += (point1[i] - point2[i]) * (point1[i] - point2[i]);
      }

      return distanceSquared;
   }



   /**
    * This class implements a Node in a KDTree.  Every point in points[][]
    * belongs to exactly one leaf Node.  Internal nodes contain split
    * information.
    */
   private class KDNode
   {
      private final double[][] points;
      private final int startIndex;
      private final int endIndex;
      private double splitValue;
      private int splitDimension;
      private KDNode leftChild;
      private KDNode rightChild;
      private boolean isLeaf;
      private double[] minExtents, maxExtents;

      public KDNode(double[][] points, int startIndex, int endIndex, int maxPointsInLeaves)
      {
         this.points = points;
         this.startIndex = startIndex;
         this.endIndex = endIndex;
         this.splitDimension = -1;
         this.splitValue = Double.NaN;
         this.leftChild = null;
         this.rightChild = null;

         int dimensions = points[0].length;

         minExtents = new double[dimensions];
         maxExtents = new double[dimensions];

         if (endIndex - startIndex + 1 > maxPointsInLeaves)
         {
            isLeaf = false;
            split(maxPointsInLeaves);
         }
         else
         {
            isLeaf = true;

            // Find the extents in this leaf
            for (int i = 0; i < dimensions; i++)
            {
               minExtents[i] = Double.POSITIVE_INFINITY;
               maxExtents[i] = Double.NEGATIVE_INFINITY;

               for (int index = startIndex; index <= endIndex; index++)
               {
                  double[] point = points[index];
                  if (point[i] > maxExtents[i])
                     maxExtents[i] = point[i];
                  if (point[i] < minExtents[i])
                     minExtents[i] = point[i];
               }
            }
         }
      }

      public void getExtents(double[] minExtent, double[] maxExtent)
      {
         for (int i = 0; i < this.minExtents.length; i++)
         {
            minExtent[i] = this.minExtents[i];
            maxExtent[i] = this.maxExtents[i];
         }
      }

      private double[] getMinExtents()
      {
         return this.minExtents;
      }

      private double[] getMaxExtents()
      {
         return this.maxExtents;
      }



      /**
       * Splits the current node into two child nodes.  Splitting happens
       * along the point dimension with the greatest width, and the split
       * value is the midpoint of the bounds along this dimension.  The
       * points are swapped in place until they all belong to the correct
       * child node.
       *
       * The parameter maxPointsInLeaves indicates at what point a Node
       * stops splitting.  It must be passed in to allow proper recursive
       * construcion of Nodes.
       */
      private void split(int maxPointsInLeaves)
      {
         setSplitParameters();
         int leftMarker = startIndex - 1;
         int rightMarker = endIndex + 1;

         // Partially sort points in place.
         while (leftMarker < rightMarker)
         {
            // Advance left marker and test
            while (points[++leftMarker][splitDimension] < splitValue)
            {
               
            }

            // Advance right marker and test
            while (points[--rightMarker][splitDimension] > splitValue)
            {
               
            }

            // Swap points and objects
            if (leftMarker < rightMarker)
            {
               double[] temp = points[leftMarker];
               points[leftMarker] = points[rightMarker];
               points[rightMarker] = temp;

               if (hasObjects)
               {
                  Object tempObject = objects[leftMarker];
                  objects[leftMarker] = objects[rightMarker];
                  objects[rightMarker] = tempObject;
               }
            }
         }

         // Now leftMarker >= rightMarker; use leftMarker to divide the points.
         if ((leftMarker > endIndex) || (rightMarker < startIndex))
         {
            throw new RuntimeException("Must test for which marker is valid and use that!");
         }

         this.leftChild = new KDNode(points, startIndex, leftMarker - 1, maxPointsInLeaves);
         this.rightChild = new KDNode(points, leftMarker, endIndex, maxPointsInLeaves);

         // Now find the extents for this node, by combining the extents of the children:
         double[] minExtentsLeft = leftChild.getMinExtents();
         double[] maxExtentsLeft = leftChild.getMaxExtents();
         double[] minExtentsRight = rightChild.getMinExtents();
         double[] maxExtentsRight = rightChild.getMaxExtents();

         for (int i = 0; i < minExtentsLeft.length; i++)
         {
            this.minExtents[i] = Math.min(minExtentsLeft[i], minExtentsRight[i]);
            this.maxExtents[i] = Math.max(maxExtentsLeft[i], maxExtentsRight[i]);
         }
      }


      /**
       * Sets the splitting dimension and the value at which to split along
       * this dimension.  Computes which dimension has the greatest distance between maximum and
       * minimum values.  This becomes our splitting dimension.  The midpoint
       * of the maximum and minimum values becomes our split value.
       *
       */
      private void setSplitParameters()
      {
         int pointDimensionality = points[0].length;
         double widestWidth = Double.NEGATIVE_INFINITY;
         for (int i = 0; i < pointDimensionality; i++)
         {
            double minValue = points[startIndex][i];
            double maxValue = minValue;

            // double averageValue = minValue;
            for (int j = startIndex + 1; j < endIndex; j++)
            {
               if (points[j][i] < minValue)
               {
                  minValue = points[j][i];
               }
               else if (points[j][i] > maxValue)
               {
                  maxValue = points[j][i];
               }

               // averageValue += points[j][i];
            }

            if (widestWidth < maxValue - minValue)
            {
               widestWidth = maxValue - minValue;
               this.splitDimension = i;

               // This divides the space equally, but not the points.
               this.splitValue = minValue + widestWidth / 2.0;

               // This divides the points roughly in half unless there are big outlying values.
               // The better but more expensive choice would be to use the median value
               // instead of the average value.
               // this.splitValue = averageValue / (endIndex-startIndex+1);
            }
         }
      }

      /**
       * Returns the closest point in points[][] to testPoint.
       *
       * @param testPoint double[]
       * @return double[]
       */
      private double[] closestPoint(double[] testPoint)
      {
         return points[closestPointIndex(testPoint)];
      }

      private double[] closestPoint(double[] testPoint, double maxDistance)
      {
         int closestIndex = this.closestPointIndex(testPoint, maxDistance, null, null);
         if (closestIndex == -1)
            return null;

         return points[closestPointIndex(testPoint)];
      }


      private double[] closestPointBruteForceSearch(double[] testPoint)
      {
         double[] closestPoint = null;
         double closestDistanceSquared = Double.POSITIVE_INFINITY;

         for (double[] point : points)
         {
            double distanceSquared = distanceSquared(testPoint, point);

            if (distanceSquared < closestDistanceSquared)
            {
               closestDistanceSquared = distanceSquared;
               closestPoint = point;
            }
         }

         return closestPoint;
      }


      /**
       * Returns the index of the closest point in points[][] to testPoint.
       *
       * @param testPoint double[]
       * @return int
       */
      private int closestPointIndex(double[] testPoint)
      {
         return closestPointIndex(testPoint, Double.POSITIVE_INFINITY, null, null);
      }

      /**
       * Returns the index of the closest point in points[][] to testPoint.
       *
       * @param testPoint double[]
       * @return int
       */
      private int closestPointIndex(double[] testPoint, double maxDistanceSquared, ArrayList<Integer> excludedIndexes, HashSet<Integer> subsetIndexes)
      {
         int closestIndex = -1;

         // If this Node is a leaf, do brute force search for the closest.
         if (this.isLeaf)
         {
            double bestDistanceSquared = Double.POSITIVE_INFINITY;

            for (int i = startIndex; i <= endIndex; i++)
            {
               if ((subsetIndexes != null) &&!subsetIndexes.contains(i))
               {
                  continue;
               }

               // Skip over excluded indexes, which represent other points that were already found.
               if ((excludedIndexes != null) && excludedIndexes.contains(i))
               {
                  continue;
               }

               double distanceSquared = distanceSquared(testPoint, points[i]);
               if ((distanceSquared < bestDistanceSquared) && (distanceSquared <= maxDistanceSquared))
               {
                  bestDistanceSquared = distanceSquared;
                  closestIndex = i;
               }
            }
         }

         else    // Otherwise, search the children.   Start with the child containing the point.
         {
            KDNode firstNode, secondNode;
            if (testPoint[this.splitDimension] < this.splitValue)
            {
               firstNode = this.leftChild;
               secondNode = this.rightChild;
            }
            else
            {
               firstNode = this.rightChild;
               secondNode = this.leftChild;
            }

            closestIndex = firstNode.closestPointIndex(testPoint, maxDistanceSquared, excludedIndexes, subsetIndexes);
            double firstDistanceSquared;
            if (closestIndex != -1)
               firstDistanceSquared = KDTree.distanceSquared(testPoint, points[closestIndex]);
            else
               firstDistanceSquared = Double.POSITIVE_INFINITY;

            // Only search the other child if testPoint is closer to splitValue than current
            // best distance.  Otherwise, the other child cannot have the closest point.
            double distanceToSplitSquared = (this.splitValue - testPoint[this.splitDimension]) * (this.splitValue - testPoint[this.splitDimension]);

            // Only search the other side if you can potentially do better than the best and potentially do better than the max allowed distance.
            if ((distanceToSplitSquared < firstDistanceSquared) && (distanceToSplitSquared < maxDistanceSquared))
            {
               // Also search the other side only if you can potentially do better based on the extents of the other side:
               double[] otherMinExtents = secondNode.getMinExtents();
               double[] otherMaxExtents = secondNode.getMaxExtents();

               int dimensions = otherMinExtents.length;

               // double[] nearestExtentPoint = new double[dimensions];
               double distanceToNearestExtentSquared = 0.0;
               for (int index = 0; index < dimensions; index++)
               {
                  if (testPoint[index] <= otherMinExtents[index])
                     distanceToNearestExtentSquared += (testPoint[index] - otherMinExtents[index]) * (testPoint[index] - otherMinExtents[index]);
                  else if (testPoint[index] >= otherMaxExtents[index])
                     distanceToNearestExtentSquared += (testPoint[index] - otherMaxExtents[index]) * (testPoint[index] - otherMaxExtents[index]);
               }

               if ((distanceToNearestExtentSquared < firstDistanceSquared) && (distanceToNearestExtentSquared < maxDistanceSquared))
               {
                  int otherClosestIndex = secondNode.closestPointIndex(testPoint, maxDistanceSquared, excludedIndexes, subsetIndexes);
                  double otherDistanceSquared;
                  if (otherClosestIndex != -1)
                     otherDistanceSquared = KDTree.distanceSquared(testPoint, points[otherClosestIndex]);
                  else
                     otherDistanceSquared = Double.POSITIVE_INFINITY;

                  if (otherDistanceSquared < firstDistanceSquared)
                  {
                     closestIndex = otherClosestIndex;
                  }
               }
            }
         }

         return closestIndex;
      }
   }


   /**
    * This method tests the KD Tree using a BDI terrain file containing an
    * unordered collection of 3D points.
    *
    * @param args String[]
    */
   public static void main(String[] args)
   {
//    String fileName = "TerrainFiles/rocks1.asc";
      String fileName = "TerrainFiles/terrainC.asc";

//    String fileName = "TerrainFiles/terrainD.asc"; //"terrainD.asc"; //

      int maxPointsInLeaves = 10;
      Random random = new Random();

      // Create and test a KDTree with lots of points.
      long treeBuildStart = System.currentTimeMillis();
      KDTree tree = new KDTree(fileName, maxPointsInLeaves);
      long treeBuildEnd = System.currentTimeMillis();
      double[][] points = tree.getPoints();
      System.out.println("It took " + (treeBuildEnd - treeBuildStart) + " millis to build the tree with " + points.length + " points.");

      double[] minExtents = new double[3];
      double[] maxExtents = new double[3];

      tree.getExtents(minExtents, maxExtents);
      Point3d minExtents3d = new Point3d(minExtents);
      Point3d maxExtents3d = new Point3d(maxExtents);

      System.out.println("minExtents = " + minExtents3d);
      System.out.println("maxExtents = " + maxExtents3d);

//    testLookupSamePointOnGrid(tree);
//
//    double maxDistance = 100.0;
//    testLookupSamePointPlusDeltaOnGrid(tree, 0.0, 1, maxDistance);
//    testLookupSamePointPlusDeltaOnGrid(tree, 0.001, 1, maxDistance);
//    testLookupSamePointPlusDeltaOnGrid(tree, 1.0, 10, maxDistance);
//    testLookupSamePointPlusDeltaOnGrid(tree, 2.0, 10, maxDistance);
//    testLookupSamePointPlusDeltaOnGrid(tree, 4.0, 100, maxDistance);
//    testLookupSamePointPlusDeltaOnGrid(tree, 8.0, 100, maxDistance);
//    testLookupSamePointPlusDeltaOnGrid(tree, 16.0, 100, maxDistance);
//    testLookupSamePointPlusDeltaOnGrid(tree, 32.0, 100, maxDistance);
//    testLookupSamePointPlusDeltaOnGrid(tree, 64.0, 200, maxDistance);
//    testLookupSamePointPlusDeltaOnGrid(tree, 128.0, 400, maxDistance);
//    testLookupSamePointPlusDeltaOnGrid(tree, 256.0, 1000, maxDistance);
//    testLookupSamePointPlusDeltaOnGrid(tree, 512.0, 1000, maxDistance);
//    testLookupSamePointPlusDeltaOnGrid(tree, 1024.0, 1000, maxDistance);
//    testLookupSamePointPlusDeltaOnGrid(tree, 2048.0, 10000, maxDistance);
//    testLookupSamePointPlusDeltaOnGrid(tree, 4096.0, 10000, maxDistance);
//    testLookupSamePointPlusDeltaOnGrid(tree, 8192.0, 10000, maxDistance);
//    testLookupSamePointPlusDeltaOnGrid(tree, 16384.0, 10000, maxDistance);
//    testLookupSamePointPlusDeltaOnGrid(tree, 1000000.0, 10000, maxDistance);
//
//    testLookupTimingOnGrid(tree, false);
//    testLookupTimingOnGrid(tree, true);

//    if (1==1) return;

//    testTree(tree, 10);

      // Prune and test a KDTree with fewer points.
      System.out.println("Pruning the Tree");
      long treePruneStart = System.currentTimeMillis();

      final double NEW_POINT_RESOLUTION = 10.0;
      tree = tree.prunePointsCopy(NEW_POINT_RESOLUTION);

      long treePruneEnd = System.currentTimeMillis();
      points = tree.getPoints();
      System.out.println("It took " + (treePruneEnd - treePruneStart) + " millis to prune the tree to " + points.length + " points.");
      testTree(tree, 10);

      int bruteNumTests = 1000;
      long bruteStart = System.currentTimeMillis();
      double[] bestPoint = null;
      for (int i = 0; i < bruteNumTests; i++)
      {
         double distance;
         int randomIndex = random.nextInt(points.length - 1);
         double[] testPoint = points[randomIndex];
         double bestDistance = Double.POSITIVE_INFINITY;
         for (int j = 0; j < points.length; j++)
         {
            distance = distanceSquared(testPoint, points[j]);

            if (distance < bestDistance)
            {
               bestDistance = distance;
               bestPoint = points[j];
            }
         }

         if ((testPoint[0] != bestPoint[0]) || (testPoint[1] != bestPoint[1]) || (testPoint[2] != bestPoint[2]))
         {
            System.err.println("Bad news: points did not match.");
         }
      }

      long bruteEnd = System.currentTimeMillis();
      System.out.println("For " + bruteNumTests + " brute force queries, number of millis = " + (bruteEnd - bruteStart));

      System.out.println("Tests done.");
   }

   private static void testTree(KDTree tree, int maxDistancePrints)
   {
      Random random = new Random();

      double[][] points = tree.getPoints();
      System.out.println("##### NUMBER OF POINTS IN TREE TO TEST: " + points.length);

      // Test some random points.  For each case, the closest point should
      // equal the test point, since the test point comes from the same
      // collection.
      long kdStart = System.currentTimeMillis();
      int kdNumTests = 1000;
      double DISTANCE_MOVE = 10.0;
      for (int i = 0; i < kdNumTests; i++)
      {
         int randomIndex = random.nextInt(points.length - 1);
         double[] testPoint = new double[] {points[randomIndex][0], points[randomIndex][1], points[randomIndex][2]};
         testPoint[0] += Math.random() * DISTANCE_MOVE;
         testPoint[1] += Math.random() * DISTANCE_MOVE;

         double[] closestPoint = tree.closestPoint(testPoint);
         double distanceSquared = distanceSquared(testPoint, closestPoint);

//         if ((i < maxDistancePrints) && DEBUG)
//         {
//            System.out.print("testPoint: (" + testPoint[0] + ", " + testPoint[1] + ", " + testPoint[2] + ")");
//            System.out.print("closestPoint: (" + closestPoint[0] + ", " + closestPoint[1] + ", " + closestPoint[2] + ")");
//            System.out.println("distance: " + Math.sqrt(distanceSquared));
//         }

         /*
          * if (testPoint[0] != closestPoint[0] ||
          *   testPoint[1] != closestPoint[1] ||
          *   testPoint[2] != closestPoint[2])
          */
         if (distanceSquared > 2.0 * DISTANCE_MOVE * DISTANCE_MOVE)
         {
            System.err.println("Bad news: points did not match.");
         }
      }

      long kdEnd = System.currentTimeMillis();

      System.out.println("For " + kdNumTests + " KD queries, number of millis = " + (kdEnd - kdStart));

      // Test code for finding a collection of points.  For each case, print out the point's value
      // and distance to the test point.
      kdStart = System.currentTimeMillis();
      kdNumTests = 1000;

      for (int i = 0; i < kdNumTests; i++)
      {
         int randomIndex = random.nextInt(points.length - 1);
         double[] testPoint = new double[] {points[randomIndex][0], points[randomIndex][1], points[randomIndex][2]};
         testPoint[0] += Math.random() * DISTANCE_MOVE;
         testPoint[1] += Math.random() * DISTANCE_MOVE;

         int numPoints = 10;
//         ArrayList<double[]> closestPoints = tree.closestPoints(testPoint, numPoints, Double.POSITIVE_INFINITY);
         for (int j = 0; j < numPoints; j++)
         {
//            double[] closestPoint = closestPoints.get(j);
//            double distanceSquared = distanceSquared(testPoint, closestPoint);
//            if ((i < maxDistancePrints) && DEBUG)
//            {
//               System.out.print("testPoint: (" + testPoint[0] + ", " + testPoint[1] + ", " + testPoint[2] + ")");
//               System.out.print("closestPoint: (" + closestPoint[0] + ", " + closestPoint[1] + ", " + closestPoint[2] + ")");
//               System.out.println("distance: " + Math.sqrt(distanceSquared));
//            }
         }
      }

      kdEnd = System.currentTimeMillis();

      System.out.println("For " + kdNumTests + " multiple-point queries, number of millis = " + (kdEnd - kdStart));
   }


   public static void testLookupTimingOnGrid(KDTree kdTree, boolean checkWithBruteForce)
   {
      int numberOfTotalRuns = 0;
      long totalStartTime = System.currentTimeMillis();

      for (double z = 0; z < 100; z += 10)
      {
         long startTime = System.currentTimeMillis();
         int numberOfRuns = 0;
         for (double x = 0; x < 600; x += 60)
         {
            for (double y = 0; y < 600; y += 60)
            {
               numberOfRuns++;
               numberOfTotalRuns++;
               double[] queryPoint = new double[] {x, y, z};

               double[] closestPoint = kdTree.closestPoint(queryPoint);

               if (checkWithBruteForce)
               {
                  double[] closestBruteForce = kdTree.closestPointBruteForceSearch(queryPoint);

                  if (closestBruteForce != closestPoint)
                  {
                     System.err.println("Doesn't match brute force!!!");

                     throw new RuntimeException("Doesn't match brute force!!!");
                  }
               }

               //
               // System.out.println("queryPoint = (" + queryPoint[0] + ", " +
               // queryPoint[1] + ", " + queryPoint[2] + ")");
               // System.out.println("closestPoint = (" + closestPoint[0] + ", " +
               // closestPoint[1] + ", " + closestPoint[2] + ")");

//             closestPoint = kdTree.closestPoint(queryPoint);
            }
         }

         long endTime = System.currentTimeMillis();

         double totalTime = (endTime - startTime);
         double averageTime = totalTime / numberOfRuns;

         System.out.println("Ran " + numberOfRuns + " lookups in " + totalTime + " ms");
         System.out.println("z = " + z + " Average time = " + averageTime);
      }

      long endTime = System.currentTimeMillis();
      double totalTime = (endTime - totalStartTime);
      double averageTime = totalTime / numberOfTotalRuns;

      System.out.println("Ran " + numberOfTotalRuns + " lookups in " + totalTime + " ms");
      System.out.println(" Average time = " + averageTime);

   }

   /**
    * Test a KDTree to make sure it returns the same point if you query for each point
    * @param kdTree KDTree
    */
   public static void testLookupSamePointOnGrid(KDTree kdTree)
   {
      long startTime = System.currentTimeMillis();
      System.out.println("Starting testLookupSamePointOnGrid");

      double[][] points = kdTree.getPoints();
      for (int i = 0; i < points.length; i++)
      {
         double[] closestPoint = kdTree.closestPoint(points[i]);
         if (points[i] != closestPoint)
         {
            Point3d point = new Point3d(points[i]);
            Point3d closest = new Point3d(closestPoint);
            System.out.println("point " + i + " = " + point + " did not match closestPoint = " + closest);

//          throw new RuntimeException("things are not working");
         }

         double[] closestPoint2 = kdTree.closestPoint(points[i]);
         if (closestPoint2 != closestPoint)
         {
            System.out.println("closestPoint2 = " + closestPoint2 + " did not match closestPoint = " + closestPoint);

            throw new RuntimeException("things are not working");
         }
      }

      System.out.println("Finished testLookupSamePointOnGrid for " + points.length);
      long endTime = System.currentTimeMillis();

      double totalTime = (endTime - startTime) * 0.001;

      System.out.println("Took " + totalTime + " seconds for " + points.length + " points.");

      double averageTime = totalTime / points.length;
      System.out.println("Average Time per query was " + averageTime * 1000.0 + " miliseconds.\n");
   }



   /**
    * Test a KDTree to make sure it gives you back the same point or a closer point if you
    * query for each point plus a small delta
    * @param kdTree KDTree
    */
   public static void testLookupSamePointPlusDeltaOnGrid(KDTree kdTree, double maxDelta, int skipPoints, double maxDistance)
   {
      long startTime = System.currentTimeMillis();
      System.out.println("Starting testLookupSamePointPlusDeltaOnGrid. maxDelta = " + maxDelta);

      double[][] points = kdTree.getPoints();

      int pointsConsidered = 0;
      for (int i = 0; i < points.length; i = i + skipPoints, pointsConsidered++)
      {
         double[] point = points[i];
         double[] adjustedPoint = new double[point.length];
         for (int j = 0; j < point.length; j++)
         {
            double delta = maxDelta * (2.0 * Math.random() - 1.0);
            adjustedPoint[j] = point[j] + delta;
         }

//       double[] closestPoint = kdTree.closestPoint(adjustedPoint, maxDistance);
         double[] closestPoint = kdTree.closestPoint(adjustedPoint);
         if ((points[i] != closestPoint) && (closestPoint != null))
         {
            // check if it is closer
            Point3d adjusted = new Point3d(adjustedPoint);
            Point3d initial = new Point3d(points[i]);
            Point3d closest = new Point3d(closestPoint);
            if (adjusted.distance(initial) <= adjusted.distance(closest))
            {
               System.out.println("point " + i + " = " + initial + " did not match adjustedPoint = " + adjusted + " instead it matched " + closest);
            }
         }
      }

      System.out.println("Finished testLookupSamePointPlusDeltaOnGrid for " + pointsConsidered);
      long endTime = System.currentTimeMillis();

      double totalTime = (endTime - startTime) * 0.001;

      System.out.println("Took " + totalTime + " seconds for " + pointsConsidered + " points.");

      double averageTime = totalTime / pointsConsidered;
      System.out.println("Average Time per query was " + averageTime * 1000.0 + " miliseconds.\n");

   }

}
