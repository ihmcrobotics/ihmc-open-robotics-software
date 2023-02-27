package us.ihmc.footstepPlanning.polygonSnapping;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.ArrayList;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

public class GarbageFreePlanarRegionPolygonSnapperTest
{
   @Test
   public void testSnapPolygonToFlatPlanarRegion()
   {
      GarbageFreePlanarRegionPolygonSnapper snapper = new GarbageFreePlanarRegionPolygonSnapper();

      ConvexPolygon2D polygonToSnap = new ConvexPolygon2D();
      polygonToSnap.addVertex(1.0, 1.0);
      polygonToSnap.addVertex(-1.0, 1.0);
      polygonToSnap.addVertex(-1.0, -1.0);
      polygonToSnap.addVertex(1.0, -1.0);
      polygonToSnap.update();

      ArrayList<ConvexPolygon2D> planarRegionConvexPolygons = new ArrayList<>();
      ConvexPolygon2D planarRegionPolygon = new ConvexPolygon2D();
      planarRegionPolygon.addVertex(10.0, 10.0);
      planarRegionPolygon.addVertex(-10.0, 10.0);
      planarRegionPolygon.addVertex(-10.0, -10.0);
      planarRegionPolygon.addVertex(10.0, -10.0);
      planarRegionPolygon.update();
      planarRegionConvexPolygons.add(planarRegionPolygon);

      RigidBodyTransform planarRegionTransformToWorld = new RigidBodyTransform();

      PlanarRegion planarRegionToSnapTo = new PlanarRegion(planarRegionTransformToWorld, planarRegionConvexPolygons);
      Point3D highestVertexInWorld = new Point3D();
      RigidBodyTransform polygonSnappingTransform = new RigidBodyTransform();
      snapper.snapPolygonToPlanarRegion(polygonToSnap, planarRegionToSnapTo, highestVertexInWorld, polygonSnappingTransform);

      RigidBodyTransform identityTransform = new RigidBodyTransform();
      assertTrue(polygonSnappingTransform.epsilonEquals(identityTransform, 1e-7));

      planarRegionTransformToWorld = new RigidBodyTransform();
      planarRegionTransformToWorld.getTranslation().set(1.2, 3.4, 7.6);

      planarRegionToSnapTo = new PlanarRegion(planarRegionTransformToWorld, planarRegionConvexPolygons);
      snapper.snapPolygonToPlanarRegion(polygonToSnap, planarRegionToSnapTo, highestVertexInWorld, polygonSnappingTransform);

      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      expectedTransform.getTranslation().set(0.0, 0.0, 7.6);
      assertTrue(polygonSnappingTransform.epsilonEquals(expectedTransform, 1e-7));
   }

   @Test
   public void testSnapPolygonToLargeRotatedRegion()
   {
      GarbageFreePlanarRegionPolygonSnapper snapper = new GarbageFreePlanarRegionPolygonSnapper();

      ConvexPolygon2D polygonToSnap = new ConvexPolygon2D();
      polygonToSnap.addVertex(1.0, 1.0);
      polygonToSnap.addVertex(-1.0, 1.0);
      polygonToSnap.addVertex(-1.0, -1.0);
      polygonToSnap.addVertex(1.0, -1.0);
      polygonToSnap.update();

      ArrayList<ConvexPolygon2D> planarRegionConvexPolygons = new ArrayList<>();
      ConvexPolygon2D planarRegionPolygon = new ConvexPolygon2D();
      planarRegionPolygon.addVertex(10.0, 10.0);
      planarRegionPolygon.addVertex(-10.0, 10.0);
      planarRegionPolygon.addVertex(-10.0, -10.0);
      planarRegionPolygon.addVertex(10.0, -10.0);
      planarRegionPolygon.update();
      planarRegionConvexPolygons.add(planarRegionPolygon);

      RigidBodyTransform planarRegionTransformToWorld = new RigidBodyTransform();
      planarRegionTransformToWorld.getTranslation().set(1.2, 3.4, 7.6);
      double roll = 0.0;
      double pitch = Math.PI / 3.0;
      double yaw = 0.0;
      planarRegionTransformToWorld.setRotationEulerAndZeroTranslation(roll, pitch, yaw);

      PlanarRegion planarRegionToSnapTo = new PlanarRegion(planarRegionTransformToWorld, planarRegionConvexPolygons);
      Point3D highestVertexInWorld = new Point3D();
      RigidBodyTransform polygonSnappingTransform = new RigidBodyTransform();
      snapper.snapPolygonToPlanarRegion(polygonToSnap, planarRegionToSnapTo, highestVertexInWorld, polygonSnappingTransform);

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

      PlanarRegionPolygonSnapperTest.assertSurfaceNormalsMatchAndSnapPreservesXFromAbove(polygonSnappingTransform, planarRegionTransformToWorld);
   }

   @Test
   public void testYawOfRegionDoesNotYawSnappedPolygon()
   {
      GarbageFreePlanarRegionPolygonSnapper snapper = new GarbageFreePlanarRegionPolygonSnapper();

      ConvexPolygon2D polygonToSnap = new ConvexPolygon2D();
      polygonToSnap.addVertex(1.0, 1.0);
      polygonToSnap.addVertex(-1.0, 1.0);
      polygonToSnap.addVertex(-1.0, -1.0);
      polygonToSnap.addVertex(1.0, -1.0);
      polygonToSnap.update();

      ArrayList<ConvexPolygon2D> planarRegionConvexPolygons = new ArrayList<>();
      ConvexPolygon2D planarRegionPolygon = new ConvexPolygon2D();
      planarRegionPolygon.addVertex(10.0, 10.0);
      planarRegionPolygon.addVertex(-10.0, 10.0);
      planarRegionPolygon.addVertex(-10.0, -10.0);
      planarRegionPolygon.addVertex(10.0, -10.0);
      planarRegionPolygon.update();
      planarRegionConvexPolygons.add(planarRegionPolygon);

      RigidBodyTransform planarRegionTransformToWorld = new RigidBodyTransform();
      planarRegionTransformToWorld.getTranslation().set(1.2, 3.4, 7.6);
      double roll = 0.0;
      double pitch = Math.PI / 3.0;
      double yaw = 0.2;
      planarRegionTransformToWorld.setRotationEulerAndZeroTranslation(roll, pitch, yaw);

      PlanarRegion planarRegionToSnapTo = new PlanarRegion(planarRegionTransformToWorld, planarRegionConvexPolygons);
      Point3D highestVertexInWorld = new Point3D();
      RigidBodyTransform polygonSnappingTransform = new RigidBodyTransform();
      snapper.snapPolygonToPlanarRegion(polygonToSnap, planarRegionToSnapTo, highestVertexInWorld, polygonSnappingTransform);

      // Make sure the two equally high vertices just got projected vertically
      Point3D highVertexOne = new Point3D(-1.0, -1.0, 0.0);
      polygonSnappingTransform.transform(highVertexOne);

      assertEquals(-1.0, highVertexOne.getX(), 1e-7);
      assertEquals(-1.0, highVertexOne.getY(), 1e-7);
      assertEquals(planarRegionToSnapTo.getPlaneZGivenXY(-1.0, -1.0), highVertexOne.getZ(), 1e-7);

      PlanarRegionPolygonSnapperTest.assertSurfaceNormalsMatchAndSnapPreservesXFromAbove(polygonSnappingTransform, planarRegionTransformToWorld);
   }
}
