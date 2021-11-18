package us.ihmc.footstepPlanning.polygonSnapping;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.util.Random;

public class HeightMapPolygonSnapperTest
{
   @Test
   public void testSnappingToPlane()
   {
      double epsilon = 1e-10;
      int numTests = 10;
      Random random = new Random(390223);

      for (int i = 0; i < numTests; i++)
      {
         Point3D pointOnPlane = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Vector3D normal = EuclidCoreRandomTools.nextVector3D(random, -1.0, 1.0, -1.0, 1.0, 0.1, 1.0);
         normal.normalize();

         Plane3D plane = new Plane3D(pointOnPlane, normal);

         ConvexPolygon2D polygonToSnap = EuclidGeometryRandomTools.nextConvexPolygon2D(random, 0.5, 3 + random.nextInt(5));
         polygonToSnap.translate(EuclidCoreRandomTools.nextDouble(random, 1.0), EuclidCoreRandomTools.nextDouble(random, 1.0));

         double gridResolution = 0.01;
         double gridSizeXY = 4.0;
         double gridCenterXY = 0.0;
         HeightMapData heightMapData = new HeightMapData(gridResolution, gridSizeXY, gridCenterXY, gridCenterXY);
         int minMaxIndexXY = heightMapData.getMinMaxIndexXY();

         for (int xIndex = 0; xIndex < 2 * minMaxIndexXY + 1; xIndex++)
         {
            for (int yIndex = 0; yIndex < 2 * minMaxIndexXY + 1; yIndex++)
            {
               double x = HeightMapTools.toCoordinate(xIndex, gridCenterXY, gridResolution, minMaxIndexXY);
               double y = HeightMapTools.toCoordinate(yIndex, gridCenterXY, gridResolution, minMaxIndexXY);
               double z = plane.getZOnPlane(x, y);
               heightMapData.setHeightAt(x, y, z);
            }
         }

         HeightMapPolygonSnapper snapper = new HeightMapPolygonSnapper();
         RigidBodyTransform snapTransform = snapper.snapPolygonToHeightMap(polygonToSnap, heightMapData);

         // Check XY position of centroid isn't changed
         Point3D centroid = new Point3D(polygonToSnap.getCentroid().getX(), polygonToSnap.getCentroid().getY(), 0.0);
         snapTransform.transform(centroid);
         Assertions.assertEquals(polygonToSnap.getCentroid().getX(), centroid.getX(), epsilon, "Centroid position not kept constant after snapping.");
         Assertions.assertEquals(polygonToSnap.getCentroid().getY(), centroid.getY(), epsilon, "Centroid position not kept constant after snapping.");

         // Check plane normal
         Vector3D zAxis = new Vector3D(Axis3D.Z);
         snapTransform.transform(zAxis);
         Assertions.assertTrue(zAxis.epsilonEquals(plane.getNormal(), epsilon), "Snap transform does not match plane normal.");

         // Check transformed x is perpendicular to world y
         Vector3D xAxis = new Vector3D(Axis3D.X);
         snapTransform.transform(xAxis);
         Assertions.assertEquals(xAxis.dot(Axis3D.Y), 0.0, epsilon, "Snap transform does not keep X along XZ plane.");
      }
   }

   @Test
   public void testBestFitSnap()
   {
      int numTests = 10;
      Random random = new Random(390223);

      for (int i = 0; i < numTests; i++)
      {
         Point3D pointOnPlane = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Vector3D normal = EuclidCoreRandomTools.nextVector3D(random, -1.0, 1.0, -1.0, 1.0, 0.1, 1.0);
         normal.normalize();

         Plane3D plane = new Plane3D(pointOnPlane, normal);

         double gridResolution = 0.01;
         double gridSizeXY = 2.0;
         double gridCenterXY = 0.0;
         HeightMapData heightMapData = new HeightMapData(gridResolution, gridSizeXY, gridCenterXY, gridCenterXY);
         int minMaxIndexXY = heightMapData.getMinMaxIndexXY();

         int centroidIndexX = HeightMapTools.toIndex(EuclidCoreRandomTools.nextDouble(random, 0.5), gridCenterXY, gridResolution, minMaxIndexXY);
         int centroidIndexY = HeightMapTools.toIndex(EuclidCoreRandomTools.nextDouble(random, 0.5), gridCenterXY, gridResolution, minMaxIndexXY);
         double centroidX = HeightMapTools.toCoordinate(centroidIndexX, gridCenterXY, gridResolution, minMaxIndexXY);
         double centroidY = HeightMapTools.toCoordinate(centroidIndexY, gridCenterXY, gridResolution, minMaxIndexXY);

         ConvexPolygon2D polygonToSnap = new ConvexPolygon2D();

         double polygonWidth = 0.5;
         polygonToSnap.addVertex(centroidX + polygonWidth, centroidY + polygonWidth);
         polygonToSnap.addVertex(centroidX - polygonWidth, centroidY + polygonWidth);
         polygonToSnap.addVertex(centroidX + polygonWidth, centroidY - polygonWidth);
         polygonToSnap.addVertex(centroidX - polygonWidth, centroidY - polygonWidth);
         polygonToSnap.update();

         double nominalZ0 = plane.getZOnPlane(polygonToSnap.getVertex(0).getX(), polygonToSnap.getVertex(0).getY());
         double nominalZ1 = plane.getZOnPlane(polygonToSnap.getVertex(1).getX(), polygonToSnap.getVertex(1).getY());
         double nominalZ2 = plane.getZOnPlane(polygonToSnap.getVertex(2).getX(), polygonToSnap.getVertex(2).getY());
         double nominalZ3 = plane.getZOnPlane(polygonToSnap.getVertex(3).getX(), polygonToSnap.getVertex(3).getY());

         double offsetZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 0.1);
         double offsetZ0 = nominalZ0 + offsetZ;
         double offsetZ1 = nominalZ1 - offsetZ;
         double offsetZ2 = nominalZ2 + offsetZ;
         double offsetZ3 = nominalZ3 - offsetZ;

         heightMapData.setHeightAt(polygonToSnap.getVertex(0).getX(), polygonToSnap.getVertex(0).getY(), offsetZ0);
         heightMapData.setHeightAt(polygonToSnap.getVertex(1).getX(), polygonToSnap.getVertex(1).getY(), offsetZ1);
         heightMapData.setHeightAt(polygonToSnap.getVertex(2).getX(), polygonToSnap.getVertex(2).getY(), offsetZ2);
         heightMapData.setHeightAt(polygonToSnap.getVertex(3).getX(), polygonToSnap.getVertex(3).getY(), offsetZ3);

         HeightMapPolygonSnapper snapper = new HeightMapPolygonSnapper();
         snapper.snapPolygonToHeightMap(polygonToSnap, heightMapData);

         Assertions.assertTrue(plane.getNormal().epsilonEquals(snapper.getBestFitPlane().getNormal(), 1e-10));
         Assertions.assertTrue(Math.abs(plane.getZOnPlane(0.0, 0.0) - snapper.getBestFitPlane().getZOnPlane(0.0, 0.0)) < 1e-10);
      }
   }
}
