package us.ihmc.footstepPlanning.swing;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.perception.gpuMapping.HeightMapTools;
import us.ihmc.perception.gpuMapping.TerrainMapData;

public class HeightMapCollisionDetector
{
   public static EuclidShape3DCollisionResult newEvaluateCollision(Box3DReadOnly collisionBox, TerrainMapData terrainMapData)
   {
      EuclidShape3DCollisionResult collisionResult = new EuclidShape3DCollisionResult();

      double resolution = terrainMapData.getCellSize();
      int centerIndex = terrainMapData.getCenterIndex();
      double centerX = terrainMapData.getGridCenterX();
      double centerY = terrainMapData.getGridCenterY();
      // get the indices of the corners of the box drawn on the ground
      // TODO switch to using the pose of the body box
      Point3DReadOnly minPoint = collisionBox.getBoundingBox().getMinPoint();
      Point3DReadOnly maxPoint = collisionBox.getBoundingBox().getMaxPoint();
      int minXIndex = HeightMapTools.coordinateToIndex(minPoint.getX(), centerX, resolution, centerIndex);
      int minYIndex = HeightMapTools.coordinateToIndex(minPoint.getY(), centerY, resolution, centerIndex);
      int maxXIndex = HeightMapTools.coordinateToIndex(maxPoint.getX(), centerX, resolution, centerIndex);
      int maxYIndex = HeightMapTools.coordinateToIndex(maxPoint.getY(), centerY, resolution, centerIndex);

      Point3D maximumPenetratingPointOnGround = new Point3D();
      Point3D maximumPenetratingPointOnBox = new Point3D();
      Vector3D maximumPenetratingBoxNormal = new Vector3D();
      double maxPenetrationDepth = 1e-4;
      double deepestPointDistanceFromCenter = Double.POSITIVE_INFINITY;

      // find the penetration depth for all the keys, which is the vertical distance
      for (int xIndex = minXIndex; xIndex <= maxXIndex; xIndex++)
      {
         for (int yIndex = minYIndex; yIndex <= maxYIndex; yIndex++)
         {
            double groundHeight = terrainMapData.getHeight(xIndex, yIndex);
            double xQuery = HeightMapTools.indexToCoordinate(xIndex, centerX, resolution, centerIndex);
            double yQuery = HeightMapTools.indexToCoordinate(yIndex, centerY, resolution, centerIndex);

            Point3D pointOnGround = new Point3D(xQuery, yQuery, groundHeight);
            Point3D pointOnBox = new Point3D();
            Vector3D boxNormal = new Vector3D();

            // find the penetration depth at this point.
            double penetrationDepth = getPointOnBoxThatsPenetrating(collisionBox, pointOnGround, pointOnBox, boxNormal);

            // don't do anything if the depth is NaN, as that means it has no intersections with the foot
            if (Double.isFinite(penetrationDepth))
            {
               if (penetrationDepth < maxPenetrationDepth)
               {
                  // the foot IS penetrating
                  maxPenetrationDepth = penetrationDepth;
                  maximumPenetratingPointOnGround.set(pointOnGround);
                  maximumPenetratingPointOnBox.set(pointOnBox);
                  maximumPenetratingBoxNormal.set(boxNormal);
                  deepestPointDistanceFromCenter = maximumPenetratingPointOnGround.distance(collisionBox.getPosition());
               }
               else if (penetrationDepth < maxPenetrationDepth + 1e-4)
               { // the foot is approximately equal to the other best penetration
                  double queryDistanceFromCenter = pointOnGround.distance(collisionBox.getPosition());
                  if (queryDistanceFromCenter < deepestPointDistanceFromCenter)
                  {  // if this query point is closer to the center of the foot, use it instead
                     maxPenetrationDepth = penetrationDepth;
                     maximumPenetratingPointOnGround.set(pointOnGround);
                     maximumPenetratingPointOnBox.set(pointOnBox);
                     maximumPenetratingBoxNormal.set(boxNormal);
                     deepestPointDistanceFromCenter = queryDistanceFromCenter;
                  }
               }
            }
         }
      }

      // check if we're intersection free. If that's the case, return the empty collision result
      if (maxPenetrationDepth > -1e-4)
         return collisionResult;

      collisionResult.setShapesAreColliding(true);
      collisionResult.setSignedDistance(maxPenetrationDepth);

      // set the collision information for the collision box (red point)
      collisionResult.getPointOnA().set(maximumPenetratingPointOnBox);
      collisionResult.getNormalOnA().set(maximumPenetratingBoxNormal);

      // set the collision information for the ground (yellow point)
      collisionResult.getPointOnB().set(maximumPenetratingPointOnGround);

      // FIXME check this.
      Vector3DReadOnly groundNormal = approximateSurfaceNormalAtPoint(maximumPenetratingPointOnGround, terrainMapData);
      collisionResult.getNormalOnB().set(groundNormal);

      return collisionResult;
   }

   public static EuclidShape3DCollisionResult evaluateCollision(Box3DReadOnly collisionBox, TerrainMapData terrainMapData)
   {
      EuclidShape3DCollisionResult collisionResult = new EuclidShape3DCollisionResult();

      double resolution = terrainMapData.getCellSize();
      int centerIndex = terrainMapData.getCenterIndex();
      double centerX = terrainMapData.getGridCenterX();
      double centerY = terrainMapData.getGridCenterY();
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
            double groundHeight = terrainMapData.getHeight(xIndex, yIndex);
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
                                                                                              terrainMapData);

         // pack these values into the results
         computeCollisionDataWhenPartialPenetration(pointOnGroundOfMaxPenetration, pointOnFootOfMaxPenetration, terrainMapData, collisionResult);
         return collisionResult;
      }
      else
      {
         collisionResult.setSignedDistance(maxPenetrationDepth);
         computeCollisionDataAtPointWhenTheWholeBottomPenetrates(maxGroundCollisionPoint, collisionBox, terrainMapData, collisionResult);

         return collisionResult;
      }
   }

   private static final double HALF_PI = 0.5 * Math.PI;
   private enum ClosestFace {FRONT, BACK, LEFT, RIGHT, BOTTOM}

   static double getPointOnBoxThatsPenetrating(Box3DReadOnly collisionBox,
                                               Point3DReadOnly pointQuery,
                                               Point3DBasics pointOnBoxToPack,
                                               Vector3DBasics normalOnBoxToPack)
   {
      // because the top of the box is open, this algorithm fails if it pitches more or less than 90 degrees
      double pitch = collisionBox.getOrientation().getPitch();
      if (pitch > HALF_PI || pitch < -HALF_PI)
      {
         throw new RuntimeException("Cannot handle a box that is pitched more than 90 degrees.");
      }

      // Transform the point to be in the box frame. This is way faster computation.
      Point3D pointInBox = new Point3D(pointQuery);
      pointInBox.applyInverseTransform(collisionBox.getPose());

      // Compute the distance to all the walls of the box, except for the top one. We know a height map can't penetrate from the top.
      // If we don't have a positive distnace to any of the walls, we know we're not in the box.
      double halfHeight = collisionBox.getSizeZ() / 2.0;
      double distanceToBottomFace = halfHeight + pointInBox.getZ();
      if (distanceToBottomFace < 0.0)
      {
         // We're not in the box
         pointOnBoxToPack.setToNaN();
         normalOnBoxToPack.setToNaN();

         return Double.NaN;
      }
      double halfLength = collisionBox.getSizeX() / 2.0;
      double distanceToFrontFace = halfLength - pointInBox.getX();
      if (distanceToFrontFace < 0.0)
      {
         // We're not in the box
         pointOnBoxToPack.setToNaN();
         normalOnBoxToPack.setToNaN();

         return Double.NaN;
      }
      double distanceToBackFace = pointInBox.getX() + halfLength;
      if (distanceToBackFace < 0.0)
      {
         // We're not in the box
         pointOnBoxToPack.setToNaN();
         normalOnBoxToPack.setToNaN();

         return Double.NaN;
      }
      double halfWidth = collisionBox.getSizeY() / 2.0;
      double distanceToLeftFace = halfWidth - pointInBox.getY();
      if (distanceToLeftFace < 0.0)
      {
         // We're not in the box
         pointOnBoxToPack.setToNaN();
         normalOnBoxToPack.setToNaN();

         return Double.NaN;
      }
      double distanceToRightFace = pointInBox.getY() + halfWidth;
      if (distanceToRightFace < 0.0)
      {
         // We're not in the box
         pointOnBoxToPack.setToNaN();
         normalOnBoxToPack.setToNaN();

         return Double.NaN;
      }

      // Get the closest face
      ClosestFace closestFace = ClosestFace.BOTTOM;
      double closestDistance = distanceToBottomFace;
      if (distanceToFrontFace < closestDistance)
      {
         closestFace = ClosestFace.FRONT;
         closestDistance = distanceToFrontFace;
      }
      if (distanceToBackFace < closestDistance)
      {
         closestFace = ClosestFace.BACK;
         closestDistance = distanceToBackFace;
      }
      if (distanceToLeftFace < closestDistance)
      {
         closestFace = ClosestFace.LEFT;
         closestDistance = distanceToLeftFace;
      }
      if (distanceToRightFace < closestDistance)
      {
         closestFace = ClosestFace.RIGHT;
         closestDistance = distanceToRightFace;
      }

      // Project the point in the box onto the closest face.
      pointOnBoxToPack.set(pointInBox);
      normalOnBoxToPack.setToZero();
      switch (closestFace)
      {
         case BOTTOM:
         {
            pointOnBoxToPack.setZ(-halfHeight);
            normalOnBoxToPack.setZ(-1.0);
            break;
         }
         case LEFT:
         {
            pointOnBoxToPack.setY(halfWidth);
            normalOnBoxToPack.setY(1.0);
            break;
         }
         case RIGHT:
         {
            pointOnBoxToPack.setY(-halfWidth);
            normalOnBoxToPack.setY(-1.0);
            break;
         }
         case FRONT:
         {
            pointOnBoxToPack.setX(halfLength);
            normalOnBoxToPack.setX(1.0);
            break;
         }
         case BACK:
         {
            pointOnBoxToPack.setX(-halfLength);
            normalOnBoxToPack.setX(-1.0);
            break;
         }
      }
      pointOnBoxToPack.applyTransform(collisionBox.getPose());
      normalOnBoxToPack.applyTransform(collisionBox.getPose());
      return -closestDistance;
   }

   static double getLowestHeightOnBoxAtPoint(Box3DReadOnly collisionBox, double xQuery, double yQuery)
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
                                                                               TerrainMapData terrainMapData,
                                                                               EuclidShape3DCollisionResult collisionResult)
   {
      Point3DReadOnly pointOnBox = getPointOnBoxWhenTheWholeBottomPenetrates(groundPoint, collisionBox);

      computeCollisionDataWhenPartialPenetration(groundPoint, pointOnBox, terrainMapData, collisionResult);
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
                                                                       TerrainMapData terrainMapData)
   {
      double closestDistance = Double.POSITIVE_INFINITY;
      double xyResolution = terrainMapData.getCellSize();

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

      int centerIndex = terrainMapData.getCenterIndex();
      double groundHeight = terrainMapData.getHeight(closestX, closestY);
      double x = HeightMapTools.indexToCoordinate(closestX, terrainMapData.getGridCenterX(), xyResolution, centerIndex);
      double y = HeightMapTools.indexToCoordinate(closestY, terrainMapData.getGridCenterY(), xyResolution, centerIndex);
      return new Point3D(x, y, groundHeight);
   }

   private static void computeCollisionDataWhenPartialPenetration(Point3DReadOnly pointOnGround,
                                                                  Point3DReadOnly pointOnBox,
                                                                  TerrainMapData terrainMapData,
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

      Vector3DReadOnly groundNormal = approximateSurfaceNormalAtPoint(pointOnGround, terrainMapData);
      collisionResult.getNormalOnB().set(groundNormal);
   }

   /**
    * Computes the average normal using the four neighboring vertices.
    */
   private static Vector3DReadOnly approximateSurfaceNormalAtPoint(Point3DReadOnly point, TerrainMapData terrainMapData)
   {
      int xIndex = HeightMapTools.coordinateToIndex(point.getX(),
                                                    terrainMapData.getGridCenterX(),
                                                    terrainMapData.getCellSize(),
                                                    terrainMapData.getCenterIndex());
      int yIndex = HeightMapTools.coordinateToIndex(point.getY(),
                                                    terrainMapData.getGridCenterY(),
                                                    terrainMapData.getCellSize(),
                                                    terrainMapData.getCenterIndex());

      Vector3D normalSum = new Vector3D();
      Vector3D firstNeighbor = new Vector3D();
      boolean firstNeighborSet = false;
      int neighborSumCount = 0;

      int cellWidth = 2 * terrainMapData.getCenterIndex() + 1;
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
                                                                terrainMapData.getGridCenterX(),
                                                                terrainMapData.getCellSize(),
                                                                terrainMapData.getCenterIndex());
            double neighbory = HeightMapTools.indexToCoordinate(neighborYIndex,
                                                                terrainMapData.getGridCenterY(),
                                                                terrainMapData.getCellSize(),
                                                                terrainMapData.getCenterIndex());
            double neighborZ = terrainMapData.getHeight(neighborXIndex, neighborYIndex);

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
