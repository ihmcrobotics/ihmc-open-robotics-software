package us.ihmc.footstepPlanning.polygonSnapping;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.ArrayList;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class PlanarRegionPolygonSnapperTest
{
   private static final int iters = 100;

   @Test
   public void testCreateTransformToMatchSurfaceNormalPreserveX()
   {
      Random random = new Random(1738L);
      for (int i = 0; i < iters; i++)
      {
         Vector3D surfaceNormal = EuclidCoreRandomTools.nextVector3D(random);
         surfaceNormal.normalize();

         Vector3D xAxis = new Vector3D();
         Vector3D yAxis = new Vector3D(0.0, 1.0, 0.0);

         xAxis.cross(yAxis, surfaceNormal);
         xAxis.normalize();
         yAxis.cross(surfaceNormal, xAxis);

         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.setColumns(xAxis, yAxis, surfaceNormal);
         RigidBodyTransform transformExpected = new RigidBodyTransform();
         transformExpected.getRotation().set(rotationMatrix);

         // Initialize this transform to have random values.
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         transform.getTranslation().setToZero();
         PolygonSnapperTools.constructRotationToMatchSurfaceNormal(surfaceNormal, transform.getRotation());

         EuclidCoreTestTools.assertEquals(transformExpected, transform, 1e-5);

         // If we create a vertical normal, after transforming it shold match the plane normal
         Vector3D normal = new Vector3D(0.0, 0.0, 1.0);
         normal.applyTransform(transform);
         EuclidCoreTestTools.assertEquals(surfaceNormal, normal, 1e-5);

         // Now do the same computation, but with a normal with non-unitary length
         normal = new Vector3D(0.0, 0.0, 1.0);
         surfaceNormal.scale(5.0);
         PolygonSnapperTools.constructRotationToMatchSurfaceNormal(surfaceNormal, transform.getRotation());
         normal.applyTransform(transform);
         surfaceNormal.normalize();

         EuclidCoreTestTools.assertEquals(surfaceNormal, normal, 1e-5);
      }
   }

   @Test
   public void testSetTranslationSettingZAndPreservingXAndY()
   {
      Random random = new Random(1738L);
      for (int i = 0; i < iters; i++)
      {
         Point3D highestVertex = EuclidCoreRandomTools.nextPoint3D(random);
         // Initialize this to a random value to ensure things get set correctly.
         RigidBodyTransform transformToModify = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         // Compute what the expected transform is.
         RigidBodyTransform transformExpected = new RigidBodyTransform(transformToModify);

         Vector3D newTranslation = new Vector3D(highestVertex.getX(), highestVertex.getY(), 0.0);
         transformExpected.transform(newTranslation);
         newTranslation.scale(-1.0);
         newTranslation.add(highestVertex);

         transformExpected.getTranslation().set(newTranslation);

         PolygonSnapperTools.setTranslationSettingZAndPreservingXAndY(highestVertex, transformToModify);

         EuclidCoreTestTools.assertEquals(transformExpected, transformToModify, 1e-5);

         // If X and Y are preserved, it should be at the original value.
         Pose3D snappedFootPose = new Pose3D(highestVertex, new Quaternion());
         snappedFootPose.getTranslation().setZ(0.0);
         snappedFootPose.applyTransform(transformToModify);

         assertEquals(highestVertex.getZ(), snappedFootPose.getZ(), 1e-5);
         assertEquals(highestVertex.getX(), snappedFootPose.getX(), 1e-5);
         assertEquals(highestVertex.getY(), snappedFootPose.getY(), 1e-5);

      }
   }

   @Test
   public void testSnapPolygonToFlatPlanarRegion()
   {
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
      RigidBodyTransform polygonSnappingTransform = PlanarRegionPolygonSnapper.snapPolygonToPlanarRegion(polygonToSnap, planarRegionToSnapTo, highestVertexInWorld);

      RigidBodyTransform identityTransform = new RigidBodyTransform();
      assertTrue(polygonSnappingTransform.epsilonEquals(identityTransform, 1e-7));

      planarRegionTransformToWorld = new RigidBodyTransform();
      planarRegionTransformToWorld.getTranslation().set(1.2, 3.4, 7.6);

      planarRegionToSnapTo = new PlanarRegion(planarRegionTransformToWorld, planarRegionConvexPolygons);
      polygonSnappingTransform = PlanarRegionPolygonSnapper.snapPolygonToPlanarRegion(polygonToSnap, planarRegionToSnapTo, highestVertexInWorld);

      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      expectedTransform.getTranslation().set(0.0, 0.0, 7.6);
      assertTrue(polygonSnappingTransform.epsilonEquals(expectedTransform, 1e-7));
   }

   @Test
   public void testSnapPolygonToLargeRotatedRegion()
   {
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

   @Test
   public void testYawOfRegionDoesNotYawSnappedPolygon()
   {
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
      EuclidCoreTestTools.assertEquals(expectedSurfaceNormal, actualSurfaceNormal, 1e-7);

      Vector3D xAxis = new Vector3D(1.0, 0.0, 0.0);
      snapTransform.transform(xAxis);
      assertEquals(0.0, xAxis.getY(), 1e-7);
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(PlanarRegionPolygonSnapper.class, PlanarRegionPolygonSnapperTest.class);
   }
}
