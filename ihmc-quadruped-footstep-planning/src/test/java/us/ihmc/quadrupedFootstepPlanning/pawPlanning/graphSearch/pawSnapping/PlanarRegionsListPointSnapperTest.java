package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.simulationConstructionSetTools.util.planarRegions.PlanarRegionsListExamples;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.fest.assertions.Fail.fail;
import static us.ihmc.robotics.Assert.*;

public class PlanarRegionsListPointSnapperTest
{
   private static final double epsilon = 1e-7;

   @Test
   public void testSimpleVerticalSnap()
   {
      Point2D pointToSnap = new Point2D();

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      generator.addCubeReferencedAtBottomMiddle(1.0, 0.5, 0.7);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      PlanarRegion planarRegion = new PlanarRegion();
      RigidBodyTransform snapTransform = PlanarRegionsListPointSnapper
            .snapPointToPlanarRegionsList(pointToSnap, planarRegionsList.getPlanarRegionsAsList(), planarRegion);

      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      expectedTransform.setTranslation(0.0, 0.0, 0.7);
      EuclidCoreTestTools.assertRigidBodyTransformEquals(expectedTransform, snapTransform, epsilon);
   }

   @Test
   public void testSimpleVerticalAndRotatedSnap()
   {
      Point2D pointToSnap = new Point2D();

      RigidBodyTransform planarRegionTransform = new RigidBodyTransform();
      planarRegionTransform.setRotationEulerAndZeroTranslation(0.1, 0.2, 0.3);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.setTransform(planarRegionTransform);

      generator.addCubeReferencedAtBottomMiddle(1.0, 0.5, 0.7);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      PlanarRegion planarRegion = new PlanarRegion();
      RigidBodyTransform snapTransform = PlanarRegionsListPointSnapper
            .snapPointToPlanarRegionsList(pointToSnap, planarRegionsList.getPlanarRegionsAsList(), planarRegion);

      assertSurfaceNormalsMatchAndSnapPreservesXFromAbove(snapTransform, planarRegionTransform);
   }

   @Test
   public void testMovingAcrossStaircase()
   {
      PlanarRegionsList planarRegionsList = PlanarRegionsListExamples.generateStairCase();
      ArrayList<double[]> xyYawToTest = new ArrayList<>();
      for (double xTranslation = 0.0; xTranslation < 3.0; xTranslation = xTranslation + 0.1)
      {
         xyYawToTest.add(new double[] {xTranslation, 0.0, 0.0});
      }

      doATest(planarRegionsList, xyYawToTest);
   }

   @Disabled
   @Test
   public void testRandomPlanarRegions()
   {
      Random random = new Random(1776L);

      int numberOfRandomObjects = 100;
      double maxX = 2.0;
      double maxY = 1.0;
      double maxZ = 0.5;

      PlanarRegionsList planarRegionsList = PlanarRegionsListExamples.generateRandomObjects(random, numberOfRandomObjects, maxX, maxY, maxZ);
      ArrayList<double[]> xyYawToTest = new ArrayList<>();

      for (double x = -maxX; x < maxX; x = x + 0.1)
      {
         for (double y = -maxY; y < maxY; y = y + 0.1)
         {
            double yaw = RandomNumbers.nextDouble(random, Math.PI);

            xyYawToTest.add(new double[] {x, y, yaw});
         }
      }

      doATest(planarRegionsList, xyYawToTest);
   }

   @Test
   public void testBumpyGround()
   {
      Random random = new Random(1776L);

      double maxX = 2.0;
      double maxY = 1.0;
      double maxZ = 0.2;

      PlanarRegionsList planarRegionsList = PlanarRegionsListExamples.generateBumpyGround(random, maxX, maxY, maxZ);
      ArrayList<double[]> xyYawToTest = new ArrayList<>();

      for (double x = -maxX; x < maxX; x = x + 0.1)
      {
         for (double y = -maxY; y < maxY; y = y + 0.1)
         {
            double yaw = RandomNumbers.nextDouble(random, Math.PI);

            xyYawToTest.add(new double[] {x, y, yaw});
         }
      }

      doATest(planarRegionsList, xyYawToTest);
   }

   @Test
   public void testSnapPolygonToFlatPlanarRegion()
   {
      Point2D pointToSnap = new Point2D();

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
      List<PlanarRegion> planarRegionList = new ArrayList<>();
      planarRegionList.add(planarRegionToSnapTo);

      PlanarRegion planarRegion = new PlanarRegion();
      RigidBodyTransform polygonSnappingTransform = PlanarRegionsListPointSnapper.snapPointToPlanarRegionsList(pointToSnap, planarRegionList, planarRegion);

      RigidBodyTransform identityTransform = new RigidBodyTransform();
      EuclidCoreTestTools.assertRigidBodyTransformEquals(identityTransform, polygonSnappingTransform, epsilon);

      planarRegionTransformToWorld = new RigidBodyTransform();
      planarRegionTransformToWorld.setTranslation(1.2, 3.4, 7.6);

      planarRegionToSnapTo = new PlanarRegion(planarRegionTransformToWorld, planarRegionConvexPolygons);
      planarRegionList.clear();
      planarRegionList.add(planarRegionToSnapTo);
      polygonSnappingTransform = PlanarRegionsListPointSnapper.snapPointToPlanarRegionsList(pointToSnap, planarRegionList, planarRegion);

      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      expectedTransform.setTranslation(0.0, 0.0, 7.6);
      EuclidCoreTestTools.assertRigidBodyTransformEquals(expectedTransform, polygonSnappingTransform, epsilon);
   }

   @Test
   public void testSnapPolygonToLargeRotatedRegion()
   {
      Point2D pointToSnap = new Point2D();

      ArrayList<ConvexPolygon2D> planarRegionConvexPolygons = new ArrayList<>();
      ConvexPolygon2D planarRegionPolygon = new ConvexPolygon2D();
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

      List<PlanarRegion> planarRegionList = new ArrayList<>();
      PlanarRegion planarRegionToSnapTo = new PlanarRegion(planarRegionTransformToWorld, planarRegionConvexPolygons);
      planarRegionList.add(planarRegionToSnapTo);

      PlanarRegion planarRegion = new PlanarRegion();
      RigidBodyTransform polygonSnappingTransform = PlanarRegionsListPointSnapper.snapPointToPlanarRegionsList(pointToSnap, planarRegionList, planarRegion);

      // Make sure the vertex just got projected vertically
      Point3D highVertexOne = new Point3D(0.0, 0.0, 0.0);
      polygonSnappingTransform.transform(highVertexOne);

      assertEquals(0.0, highVertexOne.getX(), epsilon);
      assertEquals(0.0, highVertexOne.getY(), epsilon);
      assertEquals(planarRegionToSnapTo.getPlaneZGivenXY(0.0, 0.0), highVertexOne.getZ(), epsilon);


      assertSurfaceNormalsMatchAndSnapPreservesXFromAbove(polygonSnappingTransform, planarRegionTransformToWorld);
   }

   @Test
   public void testYawOfRegionDoesNotYawSnappedPolygon()
   {
      Point2D pointToSnap = new Point2D();

      ArrayList<ConvexPolygon2D> planarRegionConvexPolygons = new ArrayList<>();
      ConvexPolygon2D planarRegionPolygon = new ConvexPolygon2D();
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

      List<PlanarRegion> planarRegionList = new ArrayList<>();
      PlanarRegion planarRegionToSnapTo = new PlanarRegion(planarRegionTransformToWorld, planarRegionConvexPolygons);
      planarRegionList.add(planarRegionToSnapTo);

      PlanarRegion planarRegion = new PlanarRegion();
      RigidBodyTransform polygonSnappingTransform = PlanarRegionsListPointSnapper.snapPointToPlanarRegionsList(pointToSnap, planarRegionList, planarRegion);

      // Make sure the two equally high vertices just got projected vertically
      Point3D highVertexOne = new Point3D(0.0, 0.0, 0.0);
      polygonSnappingTransform.transform(highVertexOne);

      assertEquals(0.0, highVertexOne.getX(), epsilon);
      assertEquals(0.0, highVertexOne.getY(), epsilon);
      assertEquals(planarRegionToSnapTo.getPlaneZGivenXY(0.0, 0.0), highVertexOne.getZ(), epsilon);

      assertSurfaceNormalsMatchAndSnapPreservesXFromAbove(polygonSnappingTransform, planarRegionTransformToWorld);
   }

   public static void assertSurfaceNormalsMatchAndSnapPreservesXFromAbove(RigidBodyTransform snapTransform, RigidBodyTransform planarRegionTransform)
   {
      Vector3D expectedSurfaceNormal = new Vector3D(0.0, 0.0, 1.0);
      planarRegionTransform.transform(expectedSurfaceNormal);

      Vector3D actualSurfaceNormal = new Vector3D(0.0, 0.0, 1.0);
      snapTransform.transform(actualSurfaceNormal);
      EuclidCoreTestTools.assertTuple3DEquals(expectedSurfaceNormal, actualSurfaceNormal, epsilon);

      Vector3D xAxis = new Vector3D(1.0, 0.0, 0.0);
      snapTransform.transform(xAxis);
      assertEquals(0.0, xAxis.getY(), epsilon);
   }

   private static void doATest(PlanarRegionsList planarRegionsList, ArrayList<double[]> xyYawToTest)
   {
      Point2DReadOnly originalPoint = new Point2D();
      RigidBodyTransform nonSnappedTransform = new RigidBodyTransform();

      for (double[] xyYaw : xyYawToTest)
      {
         Point2D pointToSnap = new Point2D(originalPoint);
         nonSnappedTransform = new RigidBodyTransform();
         nonSnappedTransform.setRotationEulerAndZeroTranslation(0.0, 0.0, xyYaw[2]);
         nonSnappedTransform.setTranslation(xyYaw[0], xyYaw[1], 0.0);
         pointToSnap.applyTransform(nonSnappedTransform, false);

         PlanarRegion planarRegionIntersection = new PlanarRegion();
         RigidBodyTransform snapTransform = PlanarRegionsListPointSnapper
               .snapPointToPlanarRegionsList(pointToSnap, planarRegionsList.getPlanarRegionsAsList(), planarRegionIntersection);

         if (snapTransform != null)
         {
            Point3D snappedVertex = new Point3D(pointToSnap.getX(), pointToSnap.getY(), 0.0);
            snapTransform.transform(snappedVertex);

            List<PlanarRegion> planarRegions = planarRegionsList
                  .findPlanarRegionsContainingPointByProjectionOntoXYPlane(snappedVertex.getX(), snappedVertex.getY());

            if (planarRegions != null)
            {
               double highestZ = Double.NEGATIVE_INFINITY;
               for (PlanarRegion planarRegion : planarRegions)
               {
                  double planeZGivenXY = planarRegion.getPlaneZGivenXY(snappedVertex.getX(), snappedVertex.getY());
                  if (planeZGivenXY > highestZ)
                     highestZ = planeZGivenXY;
               }
               assertEquals(pointToSnap.getX(), snappedVertex.getX(), epsilon);
               assertEquals(pointToSnap.getY(), snappedVertex.getY(), epsilon);
               assertEquals("planeZGivenXY = " + highestZ + ", snappedVertex.getZ() = " + snappedVertex.getZ(), highestZ, snappedVertex.getZ(),
                            epsilon);
            }
            else
            {
               fail();
            }
         }
         else
         {
            List<PlanarRegion> planarRegions = planarRegionsList
                  .findPlanarRegionsContainingPointByProjectionOntoXYPlane(pointToSnap.getX(), pointToSnap.getY());

            if (planarRegions != null)
               assertTrue(planarRegions.isEmpty());
         }
      }
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(PlanarRegionsListPointSnapper.class, PlanarRegionsListPointSnapperTest.class);
   }
}
