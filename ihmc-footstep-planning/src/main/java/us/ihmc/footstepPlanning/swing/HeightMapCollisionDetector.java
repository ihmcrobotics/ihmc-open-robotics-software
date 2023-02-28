package us.ihmc.footstepPlanning.swing;

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

      // get the indices of the corners of the box drawn on the ground
      FramePoint3DReadOnly minPoint = collisionBox.getBoundingBox().getMinPoint();
      FramePoint3DReadOnly maxPoint = collisionBox.getBoundingBox().getMaxPoint();
      int minXIndex = HeightMapTools.coordinateToIndex(minPoint.getX(), heightMap.getGridCenter().getX(), heightMap.getGridResolutionXY(), heightMap.getCenterIndex());
      int minYIndex = HeightMapTools.coordinateToIndex(minPoint.getY(), heightMap.getGridCenter().getY(), heightMap.getGridResolutionXY(), heightMap.getCenterIndex());
      int maxXIndex = HeightMapTools.coordinateToIndex(maxPoint.getX(), heightMap.getGridCenter().getX(), heightMap.getGridResolutionXY(), heightMap.getCenterIndex());
      int maxYIndex = HeightMapTools.coordinateToIndex(maxPoint.getY(), heightMap.getGridCenter().getY(), heightMap.getGridResolutionXY(), heightMap.getCenterIndex());

      double x = HeightMapTools.indexToCoordinate(minXIndex, heightMap.getGridCenter().getX(), heightMap.getGridResolutionXY(), heightMap.getCenterIndex());
      double yStart = HeightMapTools.indexToCoordinate(minYIndex, heightMap.getGridCenter().getY(), heightMap.getGridResolutionXY(), heightMap.getCenterIndex());

      Point3D collision1 = new Point3D();
      Point3D collision2 = new Point3D();

      Point3D maxGroundCollisionPoint = new Point3D();
      double maxPentrationDistance = Double.POSITIVE_INFINITY;

      for (int xIndex = minXIndex; xIndex <= maxXIndex; xIndex++)
      {
         double y = yStart;
         for (int yIndex = minYIndex; yIndex <= maxYIndex; yIndex++)
         {
            double groundHeight = heightMap.getHeightAt(xIndex, yIndex);
            // check what the vertical collisions are at this point if we were to draw a line completely vertically
            int collisions = collisionBox.intersectionWith(new Point3D(x, y, -20.0), new Vector3D(0.0, 0.0, 1.0), collision1, collision2);
            if (collisions > 0)
            {
               // we have a collision at this point!
               Point3D lowestCollisionAtPoint;
               if (collisions == 1)
                  lowestCollisionAtPoint = collision1;
               else
                  lowestCollisionAtPoint = collision1.getZ() > collision2.getZ() ? collision2 : collision1;

               double pentrationDistance = lowestCollisionAtPoint.getZ() - groundHeight;
               if (pentrationDistance < maxPentrationDistance)
               {
                  maxPentrationDistance = pentrationDistance;
                  maxGroundCollisionPoint.set(x, y, groundHeight);
               }
            }

            y += heightMap.getGridResolutionXY();
         }

         x += heightMap.getGridResolutionXY();
      }

      if (maxPentrationDistance < 0.0)
      {
         collisionResult.setSignedDistance(maxPentrationDistance);
         computeCollisionDataAtPoint(maxGroundCollisionPoint, collisionBox, heightMap, collisionResult);
      }

      return collisionResult;
   }

   private static void computeCollisionDataAtPoint(Point3DReadOnly groundPoint, FrameBox3DReadOnly collisionBox, HeightMapData heightMap, EuclidShape3DCollisionResult collisionResult)
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

      double boxLength = collisionBox.getSizeX() / 2.0;
      double boxWidth = collisionBox.getSizeY() / 2.0;
      double boxHeight = collisionBox.getSizeZ() / 2.0;


      boolean isWithinX = Math.abs(pointToProjectInLocal.getX()) < boxLength;
      boolean isWithinY = Math.abs(pointToProjectInLocal.getY()) < boxWidth;
      double xPosition = 0.0;
      double yPosition = 0.0;
      if (isWithinX != isWithinY)
      {
         xPosition = Math.signum(pointToProjectInLocal.getX()) * Math.min(boxLength, Math.abs(pointToProjectInLocal.getX()));
         yPosition = Math.signum(pointToProjectInLocal.getY()) * Math.min(boxWidth, Math.abs(pointToProjectInLocal.getY()));
      }
      else if (isWithinX)
      { // completely within
         if ((boxLength - Math.abs(pointToProjectInLocal.getX())) < (boxWidth - Math.abs(pointToProjectInLocal.getY())))
         {
            xPosition = Math.signum(pointToProjectInLocal.getX()) * boxLength;
         }
         else
         {
            yPosition = Math.signum(pointToProjectInLocal.getY()) * boxWidth;
         }
      }
      else
      {
         // completely outside
         xPosition = Math.signum(pointToProjectInLocal.getX()) * boxLength;
         yPosition = Math.signum(pointToProjectInLocal.getY()) * boxWidth;
      }
      pointToProjectInLocal.set(xPosition, yPosition, -boxHeight);

      Point3D projectedPointInWorld = new Point3D(pointToProjectInLocal);
      collisionBox.transformToWorld(projectedPointInWorld);

      return projectedPointInWorld;
   }


   /**
    * Computes the average normal using the four neighboring vertices.
    */
   private static Vector3DReadOnly approximateSurfaceNormalAtPoint(Point3DReadOnly point, HeightMapData heightMap)
   {
      int xIndex = HeightMapTools.coordinateToIndex(point.getX(), heightMap.getGridCenter().getX(), heightMap.getGridResolutionXY(), heightMap.getCenterIndex());
      int yIndex = HeightMapTools.coordinateToIndex(point.getY(), heightMap.getGridCenter().getY(), heightMap.getGridResolutionXY(), heightMap.getCenterIndex());

      Vector3D normalSum = new Vector3D();
      Vector3D firstNeighbor = new Vector3D();
      boolean firstNeighborSet = false;
      int neighborSumCount = 0;

      int cellWidth = 2 * heightMap.getCenterIndex() + 1;
      for (int xOffset : new int[]{-1, 0, 0, 1})
      {
         int neighborXIndex = xIndex + xOffset;
         if (neighborXIndex < 0 || neighborXIndex >= cellWidth)
            continue;

         for (int yOffset : new int[]{0, -1, 1, 0})
         {
            int neighborYIndex = yIndex + yOffset;

            if (neighborYIndex < 0 || neighborYIndex >= cellWidth)
               continue;

            double neighborx = HeightMapTools.indexToCoordinate(neighborXIndex, heightMap.getGridCenter().getX(), heightMap.getGridResolutionXY(), heightMap.getCenterIndex());
            double neighbory = HeightMapTools.indexToCoordinate(neighborYIndex, heightMap.getGridCenter().getY(), heightMap.getGridResolutionXY(), heightMap.getCenterIndex());
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
