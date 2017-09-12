package us.ihmc.avatar.networkProcessor.quadTreeHeightMap;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.apache.commons.lang3.mutable.MutableObject;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class HeightQuadTreeSearchTools
{
   public static HeightQuadTreeNode searchNode(HeightQuadTreeNode node, double x, double y)
   {
      return searchNode(node, (float) x, (float) y);
   }

   public static HeightQuadTreeNode searchNode(HeightQuadTreeNode node, float x, float y)
   {
      if (node == null)
         return null;

      if (node.hasChildrenArray() && Math.abs(x - node.getCenterX()) < node.getSizeX() && Math.abs(x - node.getCenterY()) < node.getSizeY())
         return node;

      if (node.hasChildrenArray() || node.getNumberOfChildren() == 0)
         return node;

      int mortonCode = 0;
      if (x > node.getCenterX())
         mortonCode |= 1;
      if (y > node.getCenterY())
         mortonCode |= 2;

      HeightQuadTreeNode child = node.getChild(mortonCode);

      if (child == null)
         return null;
      else
         return searchNode(node.getChild(mortonCode), x, y);
   }

   public static void findRadiusNeighbors(HeightQuadTreeNode rootNode, double x, double y, double radius, NeighborActionRule actionRule)
   {
      double radiusSquared = radius * radius;
      findRadiusNeighbors(rootNode, x, y, radius, radiusSquared, actionRule);
   }

   private static void findRadiusNeighbors(HeightQuadTreeNode node, double x, double y, double radius, double radiusSquared,
         NeighborActionRule actionRule)
   {
      double xNode = node.getCenterX();
      double yNode = node.getCenterY();

      // if search ball S(q,r) contains octant, simply add point indexes.
      if (contains(node, x, y, radiusSquared))
      {
         doActionOnLeavesRecursively(node, actionRule);
         return; // early pruning.
      }

      if (node.getNumberOfChildren() == 0)
      {
         double dx = x - xNode;
         double dy = y - yNode;

         double distanceSquared = dx * dx + dy * dy;
         if (distanceSquared < radiusSquared)
            actionRule.doActionOnNeighbor(node);

         return;
      }

      // check whether child nodes are in range.
      for (int childIndex = 0; childIndex < 4; childIndex++)
      {
         HeightQuadTreeNode child = node.getChild(childIndex);
         if (child == null)
            continue;

         if (!overlaps(child, x, y, radius, radiusSquared))
            continue;
         findRadiusNeighbors(child, x, y, radius, radiusSquared, actionRule);
      }
   }

   public static void doActionOnLeavesRecursively(HeightQuadTreeNode node, NeighborActionRule actionRule)
   {
      if (node.hasChildrenArray())
      {
         actionRule.doActionOnNeighbor(node);
         return;
      }

      for (int childIndex = 0; childIndex < 4; childIndex++)
      {
         HeightQuadTreeNode childNode = node.getChild(childIndex);
         if (childNode != null)
            doActionOnLeavesRecursively(childNode, actionRule);
      }
   }

   public static double findNearestNeighbor(HeightQuadTreeNode rootNode, double x, double y, double minDistance, double maxDistance,
         MutableObject<HeightQuadTreeNode> nearestNeighborToPack)
   {
      MutableDouble result = new MutableDouble(maxDistance);

      nearestNeighborToPack.setValue(null);
      findNearestNeighbor(rootNode, x, y, minDistance, result, nearestNeighborToPack);

      if (nearestNeighborToPack.getValue() != null)
         return result.doubleValue();
      else
         return Double.NaN;
   }

   private static boolean findNearestNeighbor(HeightQuadTreeNode node, double x, double y, double minDistance, MutableDouble maxDistance,
         MutableObject<HeightQuadTreeNode> nearestNeighborToPack)
   {
      double xNode = node.getCenterX();
      double yNode = node.getCenterY();

      // 1. first descend to leaf and check in leafs points.
      if (node.getNumberOfChildren() == 0)
      {
         double maxDistanceSquared = maxDistance.doubleValue() * maxDistance.doubleValue();
         double minDistanceSquared = (minDistance < 0) ? minDistance : minDistance * minDistance;

         double dx = x - xNode;
         double dy = y - yNode;

         double distanceSquared = dx * dx + dy * dy;
         boolean isBetter = distanceSquared > minDistanceSquared && distanceSquared < maxDistanceSquared;
         if (isBetter)
         {
            nearestNeighborToPack.setValue(node);
            maxDistance.setValue(Math.sqrt(distanceSquared));
         }
         return inside(node, x, y, maxDistance.doubleValue());
      }

      // determine Morton code for each point...
      int mortonCode = 0;
      if (x > xNode)
         mortonCode |= 1;
      if (y > yNode)
         mortonCode |= 2;

      HeightQuadTreeNode child = node.getChild(mortonCode);

      if (child != null)
      {
         if (findNearestNeighbor(child, x, y, minDistance, maxDistance, nearestNeighborToPack))
            return true;
      }

      // 2. if current best point completely inside, just return.
      double maxDistanceSquared = maxDistance.doubleValue() * maxDistance.doubleValue();

      // 3. check adjacent octants for overlap and check these if necessary.
      for (int childIndex = 0; childIndex < 4; childIndex++)
      {
         if (childIndex == mortonCode)
            continue;

         child = node.getChild(childIndex);

         if (child == null)
            continue;

         if (!overlaps(child, x, y, maxDistance.doubleValue(), maxDistanceSquared))
            continue;

         if (findNearestNeighbor(child, x, y, minDistance, maxDistance, nearestNeighborToPack))
            return true; // early pruning
      }

      // all children have been checked...check if point is inside the current octant...

      return inside(node, x, y, maxDistance.doubleValue());
   }

   public interface RayActionRule
   {
      public boolean doAction(double distanceFromOrigin, Point2D nodeCenter);
   }

   public static Point2D adjustCoordinatesToResolution(double x, double y, double resolution)
   {
      x = adjustCoordinatesToResolution(x, resolution);
      y = adjustCoordinatesToResolution(y, resolution);
      return new Point2D(x, y);
   }

   public static double adjustCoordinatesToResolution(double coord, double resolution)
   {
      long n = Math.round((Math.abs(coord) - 0.5 * resolution) / resolution);
      double output = n * resolution - 0.5 * resolution;
      return Math.copySign(output, coord);
   }

   public static void doActionOnRayKeys(HeightQuadTreeNode root, Point2D origin, Vector2D direction, double maxRange, RayActionRule actionRule,
         double resolution)
   {
      // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
      // basically: DDA in 3D
      origin = adjustCoordinatesToResolution(origin.getX(), origin.getY(), resolution);

      double[] directionArray = new double[2];
      double[] originArray = new double[2];
      double[] step = new double[2];
      double[] tMax = new double[2];
      double[] tDelta = new double[2];
      Point2D currentPosition = new Point2D();
      double[] currentPositionArray = new double[2];

      // Initialization phase -------------------------------------------------------
      boolean isDone = actionRule.doAction(0.0, currentPosition);

      if (isDone)
         return;

      direction.get(directionArray);
      origin.get(originArray);

      currentPosition.set(origin);
      currentPosition.get(currentPositionArray);

      for (int i = 0; i < 2; ++i)
      {
         // compute step direction
         if (directionArray[i] > 0.0)
            step[i] = resolution;
         else if (directionArray[i] < 0.0)
            step[i] = -resolution;
         else
            step[i] = 0.0;

         // compute tMax, tDelta
         if (step[i] != 0.0)
         {
            // corner point of voxel (in direction of ray)
            double voxelBorder = currentPositionArray[i];
            voxelBorder += step[i] * resolution * 0.5;

            tMax[i] = (voxelBorder - originArray[i]) / directionArray[i];
            tDelta[i] = resolution / Math.abs(directionArray[i]);
         }
         else
         {
            tMax[i] = Double.POSITIVE_INFINITY;
            tDelta[i] = Double.POSITIVE_INFINITY;
         }
      }

      // Incremental phase  ---------------------------------------------------------

      boolean done = false;
      while (!done)
      {
         int dim;

         // find minimum tMax:
         if (tMax[0] < tMax[1])
            dim = 0;
         else
            dim = 1;

         // advance in direction "dim"
         currentPositionArray[dim] += step[dim];
         tMax[dim] += tDelta[dim];

         currentPosition.set(currentPositionArray);

         // reached endpoint world coords?
         // dist_from_origin now contains the length of the ray when traveled until the border of the current voxel
         double distanceFromOrigin;// = Math.min(Math.min(tMax[0], tMax[1]), tMax[2]);

         if (tMax[0] < tMax[1])
            distanceFromOrigin = tMax[0];
         else
            distanceFromOrigin = tMax[1];
         // if this is longer than the expected ray length, we should have already hit the voxel containing the end point with the code above (key_end).
         // However, we did not hit it due to accumulating discretization errors, so this is the point here to stop the ray as we would never reach the voxel key_end
         if (distanceFromOrigin > maxRange)
         {
            done = true;
            break;
         }
         else
         { // continue to add freespace cells
            done |= actionRule.doAction(distanceFromOrigin, currentPosition);
         }
      } // end while
   }

   public static boolean contains(HeightQuadTreeNode node, double squareRadius, double x, double y)
   {
      // we exploit the symmetry to reduce the test to test whether the farthest corner is inside the search ball.
      x = Math.abs(x - node.getCenterX());
      y = Math.abs(y - node.getCenterY());

      // reminder: (x, y, z) - (-halfNodeSize, -halfNodeSize, -halfNodeSize) = (x, y, z) + (halfNodeSize, halfNodeSize, halfNodeSize)
      x += 0.5f * node.getSizeX();
      y += 0.5f * node.getSizeY();

      return (x * x + y * y) < squareRadius;
   }

   public static boolean overlaps(HeightQuadTreeNode node, double x, double y, double radius, double squareRadius)
   {
      // we exploit the symmetry to reduce the test to testing if its inside the Minkowski sum around the positive quadrant.
      double dx = Math.abs(x - node.getCenterX());
      double dy = Math.abs(y - node.getCenterY());

      // (1) Checking the line region 
      float halfSizeX = 0.5f * node.getSizeX();
      float halfSizeY = 0.5f * node.getSizeY();

      // a. completely outside, since q' is outside the relevant area.
      if (dx > radius + halfSizeX || dy > radius + halfSizeY)
         return false;

      // b. inside the line region, one of the coordinates is inside the square.
      if (dx < halfSizeX || dy < halfSizeY)
         return true;

      // (2) checking the corner region...
      dx -= halfSizeX;
      dy -= halfSizeY;

      return (dx * dx + dy * dy) < squareRadius;
   }

   public static boolean inside(HeightQuadTreeNode node, double x, double y, double radius)
   {
      // we exploit the symmetry to reduce the test to test
      // whether the farthest corner is inside the search ball.
      x = Math.abs(x - node.getCenterX());
      y = Math.abs(y - node.getCenterY());

      if (x + radius > 0.5f * node.getSizeX())
         return false;
      if (y + radius > 0.5f * node.getSizeY())
         return false;

      return true;
   }

   public static interface NeighborActionRule
   {
      public void doActionOnNeighbor(HeightQuadTreeNode node);
   }
}
