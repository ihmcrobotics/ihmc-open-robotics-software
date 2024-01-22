package us.ihmc.footstepPlanning.swing;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

public class HeightMapCollisionDetector
{
   public static EuclidShape3DCollisionResult evaluateCollision(Box3DReadOnly collisionBox, HeightMapData heightMap)
   {
      EuclidShape3DCollisionResult collisionResult = new EuclidShape3DCollisionResult();

      double resolution = heightMap.getGridResolutionXY();
      int centerIndex = heightMap.getCenterIndex();
      double centerX = heightMap.getGridCenter().getX();
      double centerY = heightMap.getGridCenter().getY();
      // get the indices of the corners of the box drawn on the ground
      // TODO switch to using the pose of the body box
      Point3DReadOnly minPoint = collisionBox.getBoundingBox().getMinPoint();
      Point3DReadOnly maxPoint = collisionBox.getBoundingBox().getMaxPoint();
      int minXIndex = HeightMapTools.coordinateToIndex(minPoint.getX(), centerX, resolution, centerIndex);
      int minYIndex = HeightMapTools.coordinateToIndex(minPoint.getY(), centerY, resolution, centerIndex);
      int maxXIndex = HeightMapTools.coordinateToIndex(maxPoint.getX(), centerX, resolution, centerIndex);
      int maxYIndex = HeightMapTools.coordinateToIndex(maxPoint.getY(), centerY, resolution, centerIndex);

      int xSpan = maxXIndex - minXIndex;
      int ySpan = maxYIndex - minYIndex;

      DMatrixRMaj penetrationDepthMap = new DMatrixRMaj(xSpan + 1, ySpan + 1);
      DMatrixRMaj penetrationDistanceMap = new DMatrixRMaj(xSpan + 1, ySpan + 1);

      Point3D maxGroundCollisionPoint = new Point3D();
      double maxPenetrationDepth = 1e-4;
      double deepestPointDistanceFromCenter = Double.POSITIVE_INFINITY;

      // tracks whether all the cells on the bottom of the foot penetrate the world
      boolean hasNonPenetratingCell = false;

      // find the penetration depth for all the keys, which is the vertical distance
      for (int xIndex = minXIndex; xIndex <= maxXIndex; xIndex++)
      {
         for (int yIndex = minYIndex; yIndex <= maxYIndex; yIndex++)
         {
            double groundHeight = heightMap.getHeightAt(xIndex, yIndex);
            double xQuery = HeightMapTools.indexToCoordinate(xIndex, centerX, resolution, centerIndex);
            double yQuery = HeightMapTools.indexToCoordinate(yIndex, centerY, resolution, centerIndex);

            // TODO check to see if this is within the body box 2D, which should account for the yaw.


            // find the penetration depth at this point
            double heightOnFootAtPoint = getLowestHeightOnBoxAtPoint(collisionBox, xQuery, yQuery);
            double penetrationDepth = heightOnFootAtPoint - groundHeight;

            // set this sdepth into the map
            int xKey = xIndex - minXIndex;
            int yKey = yIndex - minYIndex;
            penetrationDepthMap.set(xKey, yKey, penetrationDepth);

            // don't do anything if the depth is NaN, as that means it has no intersections with the foot
            if (Double.isFinite(penetrationDepth))
            {
               Point3D pointQuery = new Point3D(xQuery, yQuery, groundHeight);
               if (penetrationDepth < maxPenetrationDepth)
               {
                  // the foot IS penetrating
                  maxPenetrationDepth = penetrationDepth;
                  maxGroundCollisionPoint.set(pointQuery);
                  deepestPointDistanceFromCenter = maxGroundCollisionPoint.distance(collisionBox.getPosition());
               }
               else if (penetrationDepth < maxPenetrationDepth + 1e-4)
               { // the foot is approximately equal to the other best penetration
                  double queryDistanceFromCenter = pointQuery.distance(collisionBox.getPosition());
                  if (queryDistanceFromCenter < deepestPointDistanceFromCenter)
                  {  // if this query point is closer to the center of the foot, use it instead
                     maxPenetrationDepth = penetrationDepth;
                     maxGroundCollisionPoint.set(pointQuery);
                     deepestPointDistanceFromCenter = queryDistanceFromCenter;
                  }
               }
               else if (penetrationDepth > 1e-4)
               {
                  hasNonPenetratingCell = true;
               }
            }
         }
      }

      // check if we're intersection free. If that's the case, return the empty collision result
      if (maxPenetrationDepth > -1e-4)
         return collisionResult;

      // In this case, only part of the underside of the foot is colliding with the environment
      if (hasNonPenetratingCell)
      {
         double maxPenetrationIntoGround = Double.NEGATIVE_INFINITY;
         int xIndexOfMaxPenetration = -1;
         int yIndexOfMaxPenetration = -1;
         // compute the penetration distance
         for (int xIndex = minXIndex; xIndex <= maxXIndex; xIndex++)
         {
            for (int yIndex = minYIndex; yIndex <= maxYIndex; yIndex++)
            {
               int xKey = xIndex - minXIndex;
               int yKey = yIndex - minYIndex;
               if (penetrationDepthMap.get(xKey, yKey) > 0.0)
               {
                  // this point isn't colliding, so we don't need to consider it
                  penetrationDistanceMap.set(xKey, yKey, 0.0);
                  continue;
               }

               // this is the distance to a point on the grid that isn't intersecting
               double xyDistanceToNonPenetratingPoint = computeXYDistanceToNonPenetratingPoint(xKey, yKey, penetrationDepthMap, resolution);
               // gets the total penetration distance
               double penetrationIntoGround = EuclidCoreTools.norm(xyDistanceToNonPenetratingPoint, penetrationDepthMap.get(xKey, yKey));
               penetrationDistanceMap.set(xKey, yKey, penetrationIntoGround);
               if (penetrationIntoGround > maxPenetrationIntoGround)
               {
                  maxPenetrationIntoGround = penetrationIntoGround;
                  xIndexOfMaxPenetration = xIndex;
                  yIndexOfMaxPenetration = yIndex;
               }
            }
         }

         // get the point on the foot that this collision corresponds to.
         double xCoordinateOfMaxPenetration = HeightMapTools.indexToCoordinate(xIndexOfMaxPenetration, centerX, resolution, centerIndex);
         double yCoordinateOfMaxPenetration = HeightMapTools.indexToCoordinate(yIndexOfMaxPenetration, centerY, resolution, centerIndex);
         Point3D pointOnFootOfMaxPenetration = new Point3D(xCoordinateOfMaxPenetration,
                                                           yCoordinateOfMaxPenetration,
                                                           getLowestHeightOnBoxAtPoint(collisionBox, xCoordinateOfMaxPenetration, yCoordinateOfMaxPenetration));
         // get the point on the ground that is still intersecting, but furthest away from this penetration point
         Point3DReadOnly pointOnGroundOfMaxPenetration = computePointOnGroundOfMaxPenetration(minXIndex,
                                                                                              minYIndex,
                                                                                              xIndexOfMaxPenetration,
                                                                                              yIndexOfMaxPenetration,
                                                                                              penetrationDepthMap,
                                                                                              heightMap);

         // pack these values into the results
         computeCollisionDataWhenPartialPenetration(pointOnGroundOfMaxPenetration, pointOnFootOfMaxPenetration, heightMap, collisionResult);
         return collisionResult;
      }
      else
      {
         collisionResult.setSignedDistance(maxPenetrationDepth);
         computeCollisionDataAtPointWhenTheWholeBottomPenetrates(maxGroundCollisionPoint, collisionBox, heightMap, collisionResult);

         return collisionResult;
      }
   }

   private static double getLowestHeightOnBoxAtPoint(Box3DReadOnly collisionBox, double xQuery, double yQuery)
   {
      Point3D pointQuery = new Point3D(xQuery, yQuery, 0.0);
      Point3D collision1 = new Point3D();
      Point3D collision2 = new Point3D();
      int collisions = collisionBox.intersectionWith(pointQuery, new Vector3D(0.0, 0.0, 1.0), collision1, collision2);
      if (collisions < 1)
      {
         return Double.NaN;
      }
      else if (collisions == 1)
      {
         return collision1.getZ();
      }
      else
      {
         return Math.min(collision1.getZ(), collision2.getZ());
      }
   }

   private static void computeCollisionDataAtPointWhenTheWholeBottomPenetrates(Point3DReadOnly groundPoint,
                                                                               Box3DReadOnly collisionBox,
                                                                               HeightMapData heightMap,
                                                                               EuclidShape3DCollisionResult collisionResult)
   {
      Point3DReadOnly pointOnBox = getPointOnBoxWhenTheWholeBottomPenetrates(groundPoint, collisionBox);

      Vector3D normalAtBox = new Vector3D();
      normalAtBox.sub(pointOnBox, groundPoint);
      normalAtBox.normalize();

      collisionResult.setShapesAreColliding(true);

      // set the collision information for the collision box (red point)
      collisionResult.getPointOnA().set(pointOnBox);
      collisionResult.getNormalOnA().set(normalAtBox);

      // set the collision information for the ground (yellow point)
      collisionResult.getPointOnB().set(groundPoint);

      Vector3DReadOnly groundNormal = approximateSurfaceNormalAtPoint(groundPoint, heightMap);
      collisionResult.getNormalOnB().set(groundNormal);
   }

   static Point3DReadOnly getPointOnBoxWhenTheWholeBottomPenetrates(Point3DReadOnly groundPoint, Box3DReadOnly collisionBox)
   {
      Point3DBasics pointToProjectInLocal = new Point3D();
      collisionBox.getPose().inverseTransform(groundPoint, pointToProjectInLocal);

      Point3D intersection1 = new Point3D();
      Point3D intersection2 = new Point3D();

      int intersections = collisionBox.intersectionWith(groundPoint, new Vector3D(0.0, 0.0, 1.0), intersection1, intersection2);
      if (intersections < 1)
         return null;
      if (intersections == 1)
         return intersection1;

      return intersection1.getZ() < intersection2.getZ() ? intersection1 : intersection2;
   }

   private static double computeXYDistanceToNonPenetratingPoint(int xIndex, int yIndex, DMatrixRMaj penetrationDepthMap, double xyResolution)
   {
      double closestPoint = Double.POSITIVE_INFINITY;
      for (int x = 0; x < penetrationDepthMap.getNumRows(); x++)
      {
         for (int y = 0; y < penetrationDepthMap.getNumCols(); y++)
         {
            if (x == xIndex && y == yIndex)
               continue;
            double depth = penetrationDepthMap.get(x, y);
            if (!Double.isNaN(depth) && depth > 0.0)
            {
               int xSpan = x - xIndex;
               int ySpan = y - yIndex;
               double distance = EuclidCoreTools.norm(xSpan * xyResolution, ySpan * xyResolution);
               closestPoint = Math.min(distance, closestPoint);
            }
         }
      }

      return closestPoint;
   }

   private static Point3DReadOnly computePointOnGroundOfMaxPenetration(int xStart,
                                                                       int yStart,
                                                                       int xIndexOfMaxPenetration,
                                                                       int yIndexOfMaxPenetration,
                                                                       DMatrixRMaj penetrationDepthMap,
                                                                       HeightMapData heightMapData)
   {
      double closestDistance = Double.POSITIVE_INFINITY;
      double xyResolution = heightMapData.getGridResolutionXY();

      int closestX = -1;
      int closestY = -1;
      for (int xKey = 0; xKey < penetrationDepthMap.getNumRows(); xKey++)
      {
         for (int yKey = 0; yKey < penetrationDepthMap.getNumCols(); yKey++)
         {
            int xIndex = xKey + xStart;
            int yIndex = yKey + yStart;
            if (xIndex == xIndexOfMaxPenetration && yIndex == yIndexOfMaxPenetration)
               continue;
            double depth = penetrationDepthMap.get(xKey, yKey);
            if (!Double.isNaN(depth) && depth < 0.0) // less than zero, so that it's the collision, but closest to 0 is what we're lookng for
            {
               int xSpan = xIndex - xIndexOfMaxPenetration;
               int ySpan = yIndex - yIndexOfMaxPenetration;
               double distance = EuclidCoreTools.norm(xSpan * xyResolution, ySpan * xyResolution);
               if (distance < closestDistance)
               {
                  closestDistance = Math.min(distance, closestDistance);
                  closestX = xIndex;
                  closestY = yIndex;
               }
            }
         }
      }

      int centerIndex = heightMapData.getCenterIndex();
      double groundHeight = heightMapData.getHeightAt(closestX, closestY);
      double x = HeightMapTools.indexToCoordinate(closestX, heightMapData.getGridCenter().getX(), xyResolution, centerIndex);
      double y = HeightMapTools.indexToCoordinate(closestY, heightMapData.getGridCenter().getY(), xyResolution, centerIndex);
      return new Point3D(x, y, groundHeight);
   }

   private static void computeCollisionDataWhenPartialPenetration(Point3DReadOnly pointOnGround,
                                                                  Point3DReadOnly pointOnBox,
                                                                  HeightMapData heightMap,
                                                                  EuclidShape3DCollisionResult collisionResult)
   {
      Vector3D normalAtBox = new Vector3D();
      normalAtBox.sub(pointOnBox, pointOnGround);
      normalAtBox.normalize();

      collisionResult.setShapesAreColliding(true);

      // set the collision information for the collision box (red point)
      collisionResult.getPointOnA().set(pointOnBox);
      collisionResult.getNormalOnA().set(normalAtBox);

      // set the collision information for the ground (yellow point)
      collisionResult.getPointOnB().set(pointOnGround);

      Vector3DReadOnly groundNormal = approximateSurfaceNormalAtPoint(pointOnGround, heightMap);
      collisionResult.getNormalOnB().set(groundNormal);
   }

   /**
    * Computes the average normal using the four neighboring vertices.
    */
   private static Vector3DReadOnly approximateSurfaceNormalAtPoint(Point3DReadOnly point, HeightMapData heightMap)
   {
      int xIndex = HeightMapTools.coordinateToIndex(point.getX(),
                                                    heightMap.getGridCenter().getX(),
                                                    heightMap.getGridResolutionXY(),
                                                    heightMap.getCenterIndex());
      int yIndex = HeightMapTools.coordinateToIndex(point.getY(),
                                                    heightMap.getGridCenter().getY(),
                                                    heightMap.getGridResolutionXY(),
                                                    heightMap.getCenterIndex());

      Vector3D normalSum = new Vector3D();
      Vector3D firstNeighbor = new Vector3D();
      boolean firstNeighborSet = false;
      int neighborSumCount = 0;

      int cellWidth = 2 * heightMap.getCenterIndex() + 1;
      for (int xOffset : new int[] {-1, 0, 0, 1})
      {
         int neighborXIndex = xIndex + xOffset;
         if (neighborXIndex < 0 || neighborXIndex >= cellWidth)
            continue;

         for (int yOffset : new int[] {0, -1, 1, 0})
         {
            int neighborYIndex = yIndex + yOffset;

            if (neighborYIndex < 0 || neighborYIndex >= cellWidth)
               continue;

            double neighborx = HeightMapTools.indexToCoordinate(neighborXIndex,
                                                                heightMap.getGridCenter().getX(),
                                                                heightMap.getGridResolutionXY(),
                                                                heightMap.getCenterIndex());
            double neighbory = HeightMapTools.indexToCoordinate(neighborYIndex,
                                                                heightMap.getGridCenter().getY(),
                                                                heightMap.getGridResolutionXY(),
                                                                heightMap.getCenterIndex());
            double neighborZ = heightMap.getHeightAt(neighborXIndex, neighborYIndex);

            Point3D neighbor = new Point3D(neighborx, neighbory, neighborZ);
            if (!firstNeighborSet)
            {
               firstNeighbor.sub(neighbor, point);
               firstNeighborSet = true;
            }
            else
            {
               Vector3D secondNeighbor = new Vector3D();
               secondNeighbor.sub(neighbor, point);

               Vector3D normal = new Vector3D();
               firstNeighbor.cross(secondNeighbor, normal);
               normalSum.add(normal);
               neighborSumCount++;
            }
         }
      }

      if (neighborSumCount > 0)
         normalSum.scale(1.0 / neighborSumCount);
      else
         normalSum.set(0.0, 0.0, 1.0);

      return normalSum;
   }
}
