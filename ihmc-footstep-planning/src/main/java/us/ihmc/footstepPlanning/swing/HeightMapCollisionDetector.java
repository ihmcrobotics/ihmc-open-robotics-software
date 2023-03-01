package us.ihmc.footstepPlanning.swing;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

public class HeightMapCollisionDetector
{
   public static EuclidShape3DCollisionResult evaluateCollision(FrameBox3DReadOnly collisionBox, HeightMapData heightMap)
   {
      EuclidShape3DCollisionResult collisionResult = new EuclidShape3DCollisionResult();

      double resolution = heightMap.getGridResolutionXY();
      int centerIndex = heightMap.getCenterIndex();
      // get the indices of the corners of the box drawn on the ground
      FramePoint3DReadOnly minPoint = collisionBox.getBoundingBox().getMinPoint();
      FramePoint3DReadOnly maxPoint = collisionBox.getBoundingBox().getMaxPoint();
      int minXIndex = HeightMapTools.coordinateToIndex(minPoint.getX(), heightMap.getGridCenter().getX(), resolution, centerIndex);
      int minYIndex = HeightMapTools.coordinateToIndex(minPoint.getY(), heightMap.getGridCenter().getY(), resolution, centerIndex);
      int maxXIndex = HeightMapTools.coordinateToIndex(maxPoint.getX(), heightMap.getGridCenter().getX(), resolution, centerIndex);
      int maxYIndex = HeightMapTools.coordinateToIndex(maxPoint.getY(), heightMap.getGridCenter().getY(), resolution, centerIndex);

      int middleX = (int) Math.ceil((maxXIndex + minXIndex) / 2.0);
      int middleY = (int) Math.ceil((maxYIndex + minYIndex) / 2.0);
      int xSpan = Math.max(middleX - minXIndex, maxXIndex - middleX);
      int ySpan = Math.max(middleY - minYIndex, maxYIndex - middleY);

      DMatrixRMaj penetrationDepth = new DMatrixRMaj(2 * xSpan + 1, 2 * ySpan + 1);

      Point3D maxGroundCollisionPoint = new Point3D();
      double maxPentrationDistance = 1e-4;
      double deepestPointDistanceFromCenter = Double.POSITIVE_INFINITY;
      int worstXKey = -1;
      int worstYKey = -1;

      // do it for the middle
      double groundHeight = heightMap.getHeightAt(middleX, middleY);
      double x = HeightMapTools.indexToCoordinate(middleX, heightMap.getGridCenter().getX(), resolution, centerIndex);
      double y = HeightMapTools.indexToCoordinate(middleY, heightMap.getGridCenter().getY(), resolution, centerIndex);

      Point3D pointQuery = new Point3D(x, y, groundHeight);
      double penetrationDistance = collisionBox.signedDistance(pointQuery);
      penetrationDepth.set(xSpan, ySpan, penetrationDistance);
      if (penetrationDistance < maxPentrationDistance)
      {
         worstXKey = xSpan;
         worstYKey = ySpan;
         maxPentrationDistance = penetrationDistance;
         maxGroundCollisionPoint.set(pointQuery);
         deepestPointDistanceFromCenter = maxGroundCollisionPoint.distance(collisionBox.getPosition());
      }

      for (int yOffset = 1; yOffset <= ySpan; yOffset++)
      {
         for (int xOffset = 1; xOffset <= xSpan; xOffset++)
         {
            for (int xSign : new int[] {-1, 1})
            {
               for (int ySign : new int[] {-1, 1})
               {
                  int xIndex = middleX + xSign * xOffset;
                  int yIndex = middleY + ySign * yOffset;

                  groundHeight = heightMap.getHeightAt(xIndex, yIndex);
                  x = HeightMapTools.indexToCoordinate(xIndex, heightMap.getGridCenter().getX(), resolution, centerIndex);
                  y = HeightMapTools.indexToCoordinate(yIndex, heightMap.getGridCenter().getY(), resolution, centerIndex);

                  pointQuery = new Point3D(x, y, groundHeight);
                  penetrationDistance = collisionBox.signedDistance(pointQuery);
                  int xKey = xSpan + xSign * xOffset;
                  int yKey = ySpan + ySign * yOffset;
                  penetrationDepth.set(xKey, yKey, penetrationDistance);
                  if (penetrationDistance < maxPentrationDistance - 1e-4)
                  {
                     worstXKey = xKey;
                     worstYKey = yKey;
                     maxPentrationDistance = penetrationDistance;
                     maxGroundCollisionPoint.set(pointQuery);
                     deepestPointDistanceFromCenter = maxGroundCollisionPoint.distance(collisionBox.getPosition());
                  }
                  else if (penetrationDistance < maxPentrationDistance + 1e-4)
                  {
                     double queryDistanceFromCenter = pointQuery.distance(collisionBox.getPosition());
                     if (queryDistanceFromCenter <  deepestPointDistanceFromCenter)
                     {
                        worstXKey = xKey;
                        worstYKey = yKey;
                        maxPentrationDistance = penetrationDistance;
                        maxGroundCollisionPoint.set(pointQuery);
                        deepestPointDistanceFromCenter = queryDistanceFromCenter;
                     }
                  }
               }
            }
         }
      }

      if (maxPentrationDistance < -1e-4)
      {
         collisionResult.setSignedDistance(maxPentrationDistance);
         computeCollisionDataAtPoint(worstXKey, worstYKey, penetrationDepth, maxGroundCollisionPoint, collisionBox, heightMap, collisionResult);
      }

      return collisionResult;
   }

   private static void computeCollisionDataAtPoint(int xIndex, int yIndex, DMatrixRMaj penetrationMap,
                                                   Point3DReadOnly groundPoint,
                                                   FrameBox3DReadOnly collisionBox,
                                                   HeightMapData heightMap,
                                                   EuclidShape3DCollisionResult collisionResult)
   {
      Point3DReadOnly pointOnBox = getPointOnBox(groundPoint, collisionBox);

      Vector3D normalAtBox = new Vector3D();
      normalAtBox.sub(pointOnBox, groundPoint);
      normalAtBox.normalize();

      collisionResult.setShapesAreColliding(true);

      // set the collision information for the collision box
      collisionResult.getPointOnA().set(pointOnBox);
      collisionResult.getNormalOnA().set(normalAtBox);

      // set the collision information for the ground
      collisionResult.getPointOnB().set(groundPoint);

      Vector3DReadOnly groundNormal = approximateSurfaceNormalAtPoint(groundPoint, heightMap);
      collisionResult.getNormalOnB().set(groundNormal);
   }

   static Point3DReadOnly getPointOnBox(Point3DReadOnly groundPoint, FrameBox3DReadOnly collisionBox)
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
