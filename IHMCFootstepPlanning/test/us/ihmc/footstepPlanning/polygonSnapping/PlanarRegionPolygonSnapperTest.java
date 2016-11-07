package us.ihmc.footstepPlanning.polygonSnapping;

import static org.junit.Assert.*;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.MutationTestingTools;

public class PlanarRegionPolygonSnapperTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSnapPolygonToFlatPlanarRegion()
   {
      ConvexPolygon2d polygonToSnap = new ConvexPolygon2d();
      polygonToSnap.addVertex(1.0, 1.0);
      polygonToSnap.addVertex(-1.0, 1.0);
      polygonToSnap.addVertex(-1.0, -1.0);
      polygonToSnap.addVertex(1.0, -1.0);
      polygonToSnap.update();

      ArrayList<ConvexPolygon2d> planarRegionConvexPolygons = new ArrayList<>();
      ConvexPolygon2d planarRegionPolygon = new ConvexPolygon2d();
      planarRegionPolygon.addVertex(10.0, 10.0);
      planarRegionPolygon.addVertex(-10.0, 10.0);
      planarRegionPolygon.addVertex(-10.0, -10.0);
      planarRegionPolygon.addVertex(10.0, -10.0);
      planarRegionPolygon.update();
      planarRegionConvexPolygons.add(planarRegionPolygon);

      RigidBodyTransform planarRegionTransformToWorld = new RigidBodyTransform();

      PlanarRegion planarRegionToSnapTo = new PlanarRegion(planarRegionTransformToWorld, planarRegionConvexPolygons);
      RigidBodyTransform polygonSnappingTransform = PlanarRegionPolygonSnapper.snapPolygonToPlanarRegion(polygonToSnap, planarRegionToSnapTo);

      RigidBodyTransform identityTransform = new RigidBodyTransform();
      assertTrue(polygonSnappingTransform.epsilonEquals(identityTransform, 1e-7));

      planarRegionTransformToWorld = new RigidBodyTransform();
      planarRegionTransformToWorld.setTranslation(1.2, 3.4, 7.6);

      planarRegionToSnapTo = new PlanarRegion(planarRegionTransformToWorld, planarRegionConvexPolygons);
      polygonSnappingTransform = PlanarRegionPolygonSnapper.snapPolygonToPlanarRegion(polygonToSnap, planarRegionToSnapTo);

      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      expectedTransform.setTranslation(0.0, 0.0, 7.6);
      assertTrue(polygonSnappingTransform.epsilonEquals(expectedTransform, 1e-7));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSnapPolygonToLargeRotatedRegion()
   {
      ConvexPolygon2d polygonToSnap = new ConvexPolygon2d();
      polygonToSnap.addVertex(1.0, 1.0);
      polygonToSnap.addVertex(-1.0, 1.0);
      polygonToSnap.addVertex(-1.0, -1.0);
      polygonToSnap.addVertex(1.0, -1.0);
      polygonToSnap.update();

      ArrayList<ConvexPolygon2d> planarRegionConvexPolygons = new ArrayList<>();
      ConvexPolygon2d planarRegionPolygon = new ConvexPolygon2d();
      planarRegionPolygon.addVertex(10.0, 10.0);
      planarRegionPolygon.addVertex(-10.0, 10.0);
      planarRegionPolygon.addVertex(-10.0, -10.0);
      planarRegionPolygon.addVertex(10.0, -10.0);
      planarRegionPolygon.update();
      planarRegionConvexPolygons.add(planarRegionPolygon);

      RigidBodyTransform planarRegionTransformToWorld = new RigidBodyTransform();
      planarRegionTransformToWorld.setTranslation(1.2, 3.4, 7.6);
      double roll = 0.0;
      double pitch = Math.PI/3.0;
      double yaw = 0.0;
      planarRegionTransformToWorld.setRotationEulerAndZeroTranslation(roll, pitch, yaw);

      PlanarRegion planarRegionToSnapTo = new PlanarRegion(planarRegionTransformToWorld, planarRegionConvexPolygons);
      RigidBodyTransform polygonSnappingTransform = PlanarRegionPolygonSnapper.snapPolygonToPlanarRegion(polygonToSnap, planarRegionToSnapTo);

      // Make sure the two equally high vertices just got projected vertically
      Point3d highVertexOne = new Point3d(-1.0, -1.0, 0.0);
      polygonSnappingTransform.transform(highVertexOne);

      assertEquals(-1.0, highVertexOne.getX(), 1e-7);
      assertEquals(-1.0, highVertexOne.getY(), 1e-7);
      assertEquals(planarRegionToSnapTo.getPlaneZGivenXY(-1.0, -1.0), highVertexOne.getZ(), 1e-7);

      Point3d highVertexTwo = new Point3d(-1.0, 1.0, 0.0);
      polygonSnappingTransform.transform(highVertexTwo);

      assertEquals(-1.0, highVertexTwo.getX(), 1e-7);
      assertEquals(1.0, highVertexTwo.getY(), 1e-7);
      assertEquals(planarRegionToSnapTo.getPlaneZGivenXY(-1.0, 1.0), highVertexTwo.getZ(), 1e-7);

      // Make sure the surface normal is preserved:
      Vector3d surfaceNormalOne = new Vector3d(0.0, 0.0, 1.0);
      planarRegionTransformToWorld.transform(surfaceNormalOne);

      Vector3d surfaceNormalTwo = new Vector3d(0.0, 0.0, 1.0);
      polygonSnappingTransform.transform(surfaceNormalTwo);

      JUnitTools.assertTuple3dEquals(surfaceNormalOne, surfaceNormalTwo, 1e-7);

      // Make sure the transform has the same rotation (but no yaw)
      Vector3d eulerAngles = new Vector3d();
      polygonSnappingTransform.getRotationEuler(eulerAngles);
      assertEquals(roll, eulerAngles.getX(), 1e-5);
      assertEquals(pitch, eulerAngles.getY(), 1e-5);
      assertEquals(yaw, eulerAngles.getZ(), 1e-5);
      assertEquals(yaw, 0.0, 1e-5);

      // Make sure the xAxis has no y Component:
      Vector3d xAxis = new Vector3d(1.0, 0.0, 0.0);
      polygonSnappingTransform.transform(xAxis);
      assertEquals(0.0, xAxis.getY(), 1e-7);
   }

   public static void main(String[] args)
   {
      String targetTests = PlanarRegionPolygonSnapperTest.class.getName();
      String targetClassesInSamePackage = PlanarRegionPolygonSnapper.class.getName();
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClassesInSamePackage);
   }
}
