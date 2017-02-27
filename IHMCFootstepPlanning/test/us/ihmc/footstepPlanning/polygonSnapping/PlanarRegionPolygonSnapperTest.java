package us.ihmc.footstepPlanning.polygonSnapping;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegion;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
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
      Point3D highestVertexInWorld = new Point3D();
      RigidBodyTransform polygonSnappingTransform = PlanarRegionPolygonSnapper.snapPolygonToPlanarRegion(polygonToSnap, planarRegionToSnapTo, highestVertexInWorld);

      RigidBodyTransform identityTransform = new RigidBodyTransform();
      assertTrue(polygonSnappingTransform.epsilonEquals(identityTransform, 1e-7));

      planarRegionTransformToWorld = new RigidBodyTransform();
      planarRegionTransformToWorld.setTranslation(1.2, 3.4, 7.6);

      planarRegionToSnapTo = new PlanarRegion(planarRegionTransformToWorld, planarRegionConvexPolygons);
      polygonSnappingTransform = PlanarRegionPolygonSnapper.snapPolygonToPlanarRegion(polygonToSnap, planarRegionToSnapTo, highestVertexInWorld);

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
      double pitch = Math.PI / 3.0;
      double yaw = 0.0;
      planarRegionTransformToWorld.setRotationEulerAndZeroTranslation(roll, pitch, yaw);

      PlanarRegion planarRegionToSnapTo = new PlanarRegion(planarRegionTransformToWorld, planarRegionConvexPolygons);
      Point3D highestVertexInWorld = new Point3D();
      RigidBodyTransform polygonSnappingTransform = PlanarRegionPolygonSnapper.snapPolygonToPlanarRegion(polygonToSnap, planarRegionToSnapTo, highestVertexInWorld);

      // Make sure the two equally high vertices just got projected vertically
      Point3D highVertexOne = new Point3D(-1.0, -1.0, 0.0);
      polygonSnappingTransform.transform(highVertexOne);

      assertEquals(-1.0, highVertexOne.getX(), 1e-7);
      assertEquals(-1.0, highVertexOne.getY(), 1e-7);
      assertEquals(planarRegionToSnapTo.getPlaneZGivenXY(-1.0, -1.0), highVertexOne.getZ(), 1e-7);

      Point3D highVertexTwo = new Point3D(-1.0, 1.0, 0.0);
      polygonSnappingTransform.transform(highVertexTwo);

      assertEquals(-1.0, highVertexTwo.getX(), 1e-7);
      assertEquals(1.0, highVertexTwo.getY(), 1e-7);
      assertEquals(planarRegionToSnapTo.getPlaneZGivenXY(-1.0, 1.0), highVertexTwo.getZ(), 1e-7);

      assertSurfaceNormalsMatchAndSnapPreservesXFromAbove(polygonSnappingTransform, planarRegionTransformToWorld);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testYawOfRegionDoesNotYawSnappedPolygon()
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
      double pitch = Math.PI / 3.0;
      double yaw = 0.2;
      planarRegionTransformToWorld.setRotationEulerAndZeroTranslation(roll, pitch, yaw);

      PlanarRegion planarRegionToSnapTo = new PlanarRegion(planarRegionTransformToWorld, planarRegionConvexPolygons);
      Point3D highestVertexInWorld = new Point3D();
      RigidBodyTransform polygonSnappingTransform = PlanarRegionPolygonSnapper.snapPolygonToPlanarRegion(polygonToSnap, planarRegionToSnapTo, highestVertexInWorld);

      // Make sure the two equally high vertices just got projected vertically
      Point3D highVertexOne = new Point3D(-1.0, -1.0, 0.0);
      polygonSnappingTransform.transform(highVertexOne);

      assertEquals(-1.0, highVertexOne.getX(), 1e-7);
      assertEquals(-1.0, highVertexOne.getY(), 1e-7);
      assertEquals(planarRegionToSnapTo.getPlaneZGivenXY(-1.0, -1.0), highVertexOne.getZ(), 1e-7);

      assertSurfaceNormalsMatchAndSnapPreservesXFromAbove(polygonSnappingTransform, planarRegionTransformToWorld);
   }

   public static void assertSurfaceNormalsMatchAndSnapPreservesXFromAbove(RigidBodyTransform snapTransform, RigidBodyTransform planarRegionTransform)
   {
      Vector3D expectedSurfaceNormal = new Vector3D(0.0, 0.0, 1.0);
      planarRegionTransform.transform(expectedSurfaceNormal);

      Vector3D actualSurfaceNormal = new Vector3D(0.0, 0.0, 1.0);
      snapTransform.transform(actualSurfaceNormal);
      EuclidCoreTestTools.assertTuple3DEquals(expectedSurfaceNormal, actualSurfaceNormal, 1e-7);

      Vector3D xAxis = new Vector3D(1.0, 0.0, 0.0);
      snapTransform.transform(xAxis);
      assertEquals(0.0, xAxis.getY(), 1e-7);
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(PlanarRegionPolygonSnapper.class, PlanarRegionPolygonSnapperTest.class);
   }
}
