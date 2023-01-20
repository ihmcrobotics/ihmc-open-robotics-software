package us.ihmc.footstepPlanning.swing;

import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

public class HeightMapCollisionDetector
{
   public static EuclidShape3DCollisionResult evaluateCollision(FrameBox3DReadOnly collisionBox, HeightMapData heightMap)
   {
      EuclidShape3DCollisionResult collisionResult = new EuclidShape3DCollisionResult();

      // get the box
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

      for (int xIndex = minXIndex; xIndex <= maxXIndex; xIndex++)
      {
         double y = yStart;
         for (int yIndex = minYIndex; yIndex <= maxYIndex; yIndex++)
         {
            double groundHeight = heightMap.getHeightAt(xIndex, yIndex);
            // check what the vertical collisions are at this point.
            if (collisionBox.intersectionWith(new Point3D(x, y, -20.0), new Vector3D(0.0, 0.0, 1.0), collision1, collision2) > 0)
            {
               // we have a collision at this point!
               if (collision1.getZ() < groundHeight || collision2.getZ() < groundHeight)
               {
                  Point3D minCollision = collision1.getZ() > collision2.getZ() ? collision2 : collision1;
                  computeCollisionDataAtPoint(xIndex, yIndex, collisionBox, heightMap, collisionResult, minCollision);
               }
            }

            y += heightMap.getGridResolutionXY();
         }

         x += heightMap.getGridResolutionXY();
      }

      return collisionResult;
   }

   private static void computeCollisionDataAtPoint(int xIndex, int yIndex, FrameBox3DReadOnly collisionBox, HeightMapData heightMap, EuclidShape3DCollisionResult collisionResult, Point3DReadOnly minCollision)
   {
      double z = heightMap.getHeightAt(xIndex, yIndex);
      double penetrationDistance = minCollision.getZ() - z;

      // not pentrating as far, so go ahead and return
      if (penetrationDistance > collisionResult.getSignedDistance())
         return;

      double x = HeightMapTools.indexToCoordinate(xIndex, heightMap.getGridCenter().getX(), heightMap.getGridResolutionXY(), heightMap.getCenterIndex());
      double y = HeightMapTools.indexToCoordinate(yIndex, heightMap.getGridCenter().getY(), heightMap.getGridResolutionXY(), heightMap.getCenterIndex());

      Point3D point = new Point3D(x, y, z);
      Point3D pointOnBox = new Point3D();

      collisionBox.orthogonalProjection(point, pointOnBox);
      Vector3D normalAtBox = new Vector3D();
      normalAtBox.sub(pointOnBox, point);
      normalAtBox.normalize();

      collisionResult.setSignedDistance(penetrationDistance);
      collisionResult.setShapesAreColliding(true);

      // set the collision information for the collision box
      collisionResult.getPointOnA().set(pointOnBox);
      collisionResult.getNormalOnA().set(normalAtBox);

      // set the collision information for the ground
      collisionResult.getPointOnB().set(point);

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

      collisionResult.getNormalOnB().set(normalSum);
   }
}
