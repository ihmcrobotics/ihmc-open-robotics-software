package us.ihmc.footstepPlanning.polygonSnapping;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.util.Random;

public class HeightMapPolygonSnapperTest
{
   @Test
   public void testSnappingToPlane()
   {
      double epsilon = 1e-10;
      double normalEpsilon = 5e-2;
      int numTests = 10;
      Random random = new Random(390223);

      for (int i = 0; i < numTests; i++)
      {
         Point3D pointOnPlane = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Vector3D normal = EuclidCoreRandomTools.nextVector3D(random, -1.0, 1.0, -1.0, 1.0, 0.1, 1.0);
         normal.normalize();

         Plane3D plane = new Plane3D(pointOnPlane, normal);

         // TODO make this not assume 4 points
//         ConvexPolygon2D polygonToSnap = EuclidGeometryRandomTools.nextConvexPolygon2D(random, 0.5, 3 + random.nextInt(5));
         ConvexPolygon2D polygonToSnap = EuclidGeometryRandomTools.nextConvexPolygon2D(random, 0.5, 4);
         while (polygonToSnap.getNumberOfVertices() < 4)
         {
            polygonToSnap.addVertex(EuclidCoreRandomTools.nextPoint2D(random, 0.5));
            polygonToSnap.update();
         }
         polygonToSnap.translate(EuclidCoreRandomTools.nextDouble(random, 1.0), EuclidCoreRandomTools.nextDouble(random, 1.0));

         double gridResolution = 0.01;
         double gridSizeXY = 4.0;
         double gridCenterXY = 0.0;
         HeightMapData heightMapData = new HeightMapData(gridResolution, gridSizeXY, gridCenterXY, gridCenterXY);
         int centerIndex = heightMapData.getCenterIndex();
         int cellsPerAxis = 2 * centerIndex + 1;
         int N = cellsPerAxis * cellsPerAxis;

         for (int key = 0; key < N; key++)
         {
            double x = HeightMapTools.keyToXCoordinate(key, gridCenterXY, gridResolution, centerIndex);
            double y = HeightMapTools.keyToYCoordinate(key, gridCenterXY, gridResolution, centerIndex);
            double z = plane.getZOnPlane(x, y);
            heightMapData.setHeightAt(x, y, z);
         }

         HeightMapPolygonSnapper snapper = new HeightMapPolygonSnapper();
         FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();
         environmentHandler.setHeightMap(heightMapData);
         RigidBodyTransform snapTransform = snapper.snapPolygonToHeightMap(polygonToSnap, environmentHandler, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

         // Check XY position of centroid isn't changed
         Point3D centroid = new Point3D(polygonToSnap.getCentroid().getX(), polygonToSnap.getCentroid().getY(), 0.0);
         snapTransform.transform(centroid);
         Assertions.assertEquals(polygonToSnap.getCentroid().getX(), centroid.getX(), epsilon, "Centroid position not kept constant after snapping.");
         Assertions.assertEquals(polygonToSnap.getCentroid().getY(), centroid.getY(), epsilon, "Centroid position not kept constant after snapping.");

         // Check plane normal
         Vector3D zAxis = new Vector3D(Axis3D.Z);
         snapTransform.transform(zAxis);
         EuclidCoreTestTools.assertEquals(plane.getNormal(), zAxis, normalEpsilon);

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
         int centerIndex = heightMapData.getCenterIndex();

         int centroidIndexX = HeightMapTools.coordinateToIndex(EuclidCoreRandomTools.nextDouble(random, 0.5), gridCenterXY, gridResolution, centerIndex);
         int centroidIndexY = HeightMapTools.coordinateToIndex(EuclidCoreRandomTools.nextDouble(random, 0.5), gridCenterXY, gridResolution, centerIndex);
         double centroidX = HeightMapTools.indexToCoordinate(centroidIndexX, gridCenterXY, gridResolution, centerIndex);
         double centroidY = HeightMapTools.indexToCoordinate(centroidIndexY, gridCenterXY, gridResolution, centerIndex);

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
         FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();
         environmentHandler.setHeightMap(heightMapData);
         snapper.snapPolygonToHeightMap(polygonToSnap, environmentHandler, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

         Assertions.assertTrue(plane.getNormal().epsilonEquals(snapper.getBestFitPlane().getNormal(), 1e-10));
         Assertions.assertTrue(Math.abs(plane.getZOnPlane(0.0, 0.0) - snapper.getBestFitPlane().getZOnPlane(0.0, 0.0)) < 1e-10);
      }
   }

   @Test
   public void testAreaWhenFootOverhangs()
   {
      double gridResolution = 0.05;
      double gridSizeXY = 0.3;
      double gridCenterXY = 0.0;
      HeightMapData heightMapData = new HeightMapData(gridResolution, gridSizeXY, gridCenterXY, gridCenterXY);

      for (double x = -0.10; x <= 0.1; x += gridResolution)
      {
         for (double y = -0.10; y <= 0.1; y += gridResolution)
         {
            heightMapData.setHeightAt(x, y, 0.2);
         }
      }

      ConvexPolygon2D polygonToSnap = new ConvexPolygon2D();

      double footLength = 0.2;
      double footWidth = 0.1;
      polygonToSnap.addVertex(footLength / 2.0, footWidth / 2.0);
      polygonToSnap.addVertex(footLength / 2.0, -footWidth / 2.0);
      polygonToSnap.addVertex(-footLength / 2.0, -footWidth / 2.0);
      polygonToSnap.addVertex(-footLength / 2.0, footWidth / 2.0);
      polygonToSnap.update();

      HeightMapPolygonSnapper snapper = new HeightMapPolygonSnapper();
      FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();
      environmentHandler.setHeightMap(heightMapData);
      snapper.snapPolygonToHeightMap(polygonToSnap, environmentHandler, 0.05, Math.toRadians(45.0));

      Assertions.assertTrue(snapper.getAreaFraction() >= polygonToSnap.getArea());

      // make the foot overhang by a fair bit
      polygonToSnap.translate(-gridResolution, 0.0);
      snapper.snapPolygonToHeightMap(polygonToSnap, environmentHandler, 0.05, Math.toRadians(45.0));
      Assertions.assertFalse(snapper.getAreaFraction() >= polygonToSnap.getArea());
      Assertions.assertEquals(snapper.getAreaFraction(), (footLength - 0.05) * footWidth, 2e-3);
   }
}
