package us.ihmc.footstepPlanning.polygonSnapping;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.simulationConstructionSetTools.util.planarRegions.PlanarRegionsListExamples;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertTrue;

public class GarbageFreePlanarRegionsListPolygonSnapperTest
{
   @Test
   public void testSimpleVerticalSnap()
   {
      GarbageFreePlanarRegionListPolygonSnapper snapper = new GarbageFreePlanarRegionListPolygonSnapper();
      boolean visualize = false;
      ConvexPolygon2D polygonToSnap = PlanarRegionsListExamples.createRectanglePolygon(0.5, 0.25);
      RigidBodyTransform nonSnappedTransform = new RigidBodyTransform();

      PolygonSnapperVisualizer polygonSnapperVisualizer = null;
      if (visualize)
      {
         polygonSnapperVisualizer = new PolygonSnapperVisualizer(polygonToSnap);
      }

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      generator.addCubeReferencedAtBottomMiddle(1.0, 0.5, 0.7);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      PlanarRegion planarRegion = new PlanarRegion();
      RigidBodyTransform snapTransform = new RigidBodyTransform();
      snapper.snapPolygonToPlanarRegionsList(polygonToSnap, planarRegionsList.getPlanarRegionsAsList(), Double.POSITIVE_INFINITY, planarRegion, snapTransform);

      if (polygonSnapperVisualizer != null)
      {
         polygonSnapperVisualizer.addPlanarRegionsList(planarRegionsList, YoAppearance.Gray());
         polygonSnapperVisualizer.setSnappedPolygon(nonSnappedTransform, snapTransform);
      }

      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      expectedTransform.getTranslation().set(0.0, 0.0, 0.7);
      assertTrue(expectedTransform.epsilonEquals(snapTransform, 1e-7));

      if (visualize)
      {
         ThreadTools.sleepForever();
      }
   }

   @Test
   public void testSimpleVerticalAndRotatedSnap()
   {
      GarbageFreePlanarRegionListPolygonSnapper snapper = new GarbageFreePlanarRegionListPolygonSnapper();

      boolean visualize = false;
      ConvexPolygon2D polygonToSnap = PlanarRegionsListExamples.createRectanglePolygon(0.5, 0.25);
      RigidBodyTransform nonSnappedTransform = new RigidBodyTransform();

      PolygonSnapperVisualizer polygonSnapperVisualizer = null;
      if (visualize)
      {
         polygonSnapperVisualizer = new PolygonSnapperVisualizer(polygonToSnap);
      }

      RigidBodyTransform planarRegionTransform = new RigidBodyTransform();
      planarRegionTransform.setRotationEulerAndZeroTranslation(0.1, 0.2, 0.3);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.setTransform(planarRegionTransform);

      generator.addCubeReferencedAtBottomMiddle(1.0, 0.5, 0.7);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      PlanarRegion planarRegion = new PlanarRegion();
      RigidBodyTransform snapTransform = new RigidBodyTransform();
      snapper.snapPolygonToPlanarRegionsList(polygonToSnap, planarRegionsList.getPlanarRegionsAsList(), Double.POSITIVE_INFINITY, planarRegion, snapTransform);

      if (polygonSnapperVisualizer != null)
      {
         polygonSnapperVisualizer.addPlanarRegionsList(planarRegionsList, YoAppearance.Gray());
         polygonSnapperVisualizer.setSnappedPolygon(nonSnappedTransform, snapTransform);
      }

      PlanarRegionPolygonSnapperTest.assertSurfaceNormalsMatchAndSnapPreservesXFromAbove(snapTransform, planarRegionTransform);

      if (visualize)
      {
         ThreadTools.sleepForever();
      }
   }

   @Test
   public void testMovingAcrossStaircase()
   {
      boolean visualize = false;
      PlanarRegionsList planarRegionsList = PlanarRegionsListExamples.generateStairCase();
      ArrayList<double[]> xyYawToTest = new ArrayList<>();
      for (double xTranslation = 0.0; xTranslation < 3.0; xTranslation = xTranslation + 0.1)
      {
         xyYawToTest.add(new double[] { xTranslation, 0.0, 0.0 });
      }

      doATest(planarRegionsList, xyYawToTest, visualize);
   }

   @Test
   public void testRandomPlanarRegions()
   {
      Random random = new Random(1776L);

      boolean visualize = false;
      int numberOfRandomObjects = 100;
      double maxX = 2.0;
      double maxY = 1.0;
      double maxZ = 0.5;

      PlanarRegionsList planarRegionsList = PlanarRegionsListExamples.generateRandomObjects(random, numberOfRandomObjects, maxX, maxY, maxZ);
      ArrayList<double[]> xyYawToTest = new ArrayList<>();

      for (double x = -maxX; x<maxX; x = x + 0.1)
      {
         for (double y = -maxY; y<maxY; y = y + 0.1)
         {
            double yaw = RandomNumbers.nextDouble(random, Math.PI);

            xyYawToTest.add(new double[] { x, y, yaw });
         }
      }

      doATest(planarRegionsList, xyYawToTest, visualize);
   }

   @Test
   public void testBumpyGround()
   {
      Random random = new Random(1776L);

      boolean visualize = false;
      double maxX = 2.0;
      double maxY = 1.0;
      double maxZ = 0.2;

      PlanarRegionsList planarRegionsList = PlanarRegionsListExamples.generateBumpyGround(random, maxX, maxY, maxZ);
      ArrayList<double[]> xyYawToTest = new ArrayList<>();

      for (double x = -maxX; x<maxX; x = x + 0.1)
      {
         for (double y = -maxY; y<maxY; y = y + 0.1)
         {
            double yaw = RandomNumbers.nextDouble(random, Math.PI);

            xyYawToTest.add(new double[] { x, y, yaw });
         }
      }

      doATest(planarRegionsList, xyYawToTest, visualize);
   }

   private static void doATest(PlanarRegionsList planarRegionsList, ArrayList<double[]> xyYawToTest, boolean visualize)
   {
      ConvexPolygon2D originalPolygon = PlanarRegionsListExamples.createRectanglePolygon(0.3, 0.15);
      GarbageFreePlanarRegionListPolygonSnapper snapper = new GarbageFreePlanarRegionListPolygonSnapper();

      RigidBodyTransform nonSnappedTransform = new RigidBodyTransform();

      PolygonSnapperVisualizer polygonSnapperVisualizer = null;
      if (visualize)
      {
         polygonSnapperVisualizer = new PolygonSnapperVisualizer(originalPolygon);
      }

      if (polygonSnapperVisualizer != null)
      {
         polygonSnapperVisualizer.addPlanarRegionsList(planarRegionsList, YoAppearance.Gold(), YoAppearance.Purple(), YoAppearance.Brown(), YoAppearance.Blue(), YoAppearance.Chartreuse());
      }

      for (double[] xyYaw : xyYawToTest)
      {
         ConvexPolygon2D polygonToSnap = new ConvexPolygon2D(originalPolygon);
         nonSnappedTransform = new RigidBodyTransform();
         nonSnappedTransform.setRotationEulerAndZeroTranslation(0.0, 0.0, xyYaw[2]);
         nonSnappedTransform.getTranslation().set(xyYaw[0], xyYaw[1], 0.0);
         polygonToSnap.applyTransform(nonSnappedTransform, false);

         PlanarRegion planarRegionIntersection = new PlanarRegion();
         RigidBodyTransform snapTransform = new RigidBodyTransform();

         if (snapper.snapPolygonToPlanarRegionsList(polygonToSnap, planarRegionsList.getPlanarRegionsAsList(), Double.POSITIVE_INFINITY, planarRegionIntersection, snapTransform))
         {
            int numberOfVertices = polygonToSnap.getNumberOfVertices();
            for (int vertexIndex = 0; vertexIndex < numberOfVertices; vertexIndex++)
            {
               Point2DReadOnly vertex = polygonToSnap.getVertex(vertexIndex);
               Point3D snappedVertex = new Point3D(vertex.getX(), vertex.getY(), 0.0);

               snapTransform.transform(snappedVertex);

               List<PlanarRegion> planarRegions = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(snappedVertex.getX(), snappedVertex.getY());

               if (planarRegions != null)
               {
                  for (PlanarRegion planarRegion : planarRegions)
                  {
                     double planeZGivenXY = planarRegion.getPlaneZGivenXY(snappedVertex.getX(), snappedVertex.getY());
                     //                     assertTrue("planeZGivenXY = " + planeZGivenXY + ", snappedVertex.getZ() = " + snappedVertex.getZ(), planeZGivenXY <= snappedVertex.getZ() + 1e-4);
                  }
               }
            }
         }

         if (polygonSnapperVisualizer != null)
         {
            polygonSnapperVisualizer.setSnappedPolygon(nonSnappedTransform, snapTransform);
         }
      }

      if (visualize)
      {
         polygonSnapperVisualizer.cropBuffer();
         ThreadTools.sleepForever();
      }
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(PlanarRegionsListPolygonSnapper.class, GarbageFreePlanarRegionsListPolygonSnapperTest.class);
   }
}
