package us.ihmc.footstepPlanning.polygonSnapping;

import static org.junit.Assert.assertTrue;

import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.MutationTestingTools;
import us.ihmc.tools.thread.ThreadTools;

public class PlanarRegionsListPolygonSnapperTest
{
   private boolean VISUALIZE = false;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleVerticalSnap()
   {
      ConvexPolygon2d polygonToSnap = createRectanglePolygon(0.5, 0.25);
      RigidBodyTransform nonSnappedTransform = new RigidBodyTransform();

      PolygonSnapperVisualizer polygonSnapperVisualizer = null;
      if (VISUALIZE)
      {
         polygonSnapperVisualizer = new PolygonSnapperVisualizer(polygonToSnap);
      }

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      generator.addCubeReferencedAtBottomMiddle(1.0, 0.5, 0.7);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();


      RigidBodyTransform snapTransform = PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(polygonToSnap, planarRegionsList);

      if (polygonSnapperVisualizer != null)
      {
         polygonSnapperVisualizer.addPlanarRegionsList(planarRegionsList, YoAppearance.Gray());
         polygonSnapperVisualizer.setSnappedPolygon(nonSnappedTransform, snapTransform);
      }

      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      expectedTransform.setTranslation(0.0, 0.0, 0.7);
      assertTrue(expectedTransform.epsilonEquals(snapTransform, 1e-7));

      if (VISUALIZE)
      {
         ThreadTools.sleepForever();
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleVerticalAndRotatedSnap()
   {
      ConvexPolygon2d polygonToSnap = createRectanglePolygon(0.5, 0.25);
      RigidBodyTransform nonSnappedTransform = new RigidBodyTransform();

      PolygonSnapperVisualizer polygonSnapperVisualizer = null;
      if (VISUALIZE)
      {
         polygonSnapperVisualizer = new PolygonSnapperVisualizer(polygonToSnap);
      }

      RigidBodyTransform planarRegionTransform = new RigidBodyTransform();
      planarRegionTransform.setRotationEulerAndZeroTranslation(0.1, 0.2, 0.3);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.setTransform(planarRegionTransform);

      generator.addCubeReferencedAtBottomMiddle(1.0, 0.5, 0.7);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      RigidBodyTransform snapTransform = PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(polygonToSnap, planarRegionsList);

      if (polygonSnapperVisualizer != null)
      {
         polygonSnapperVisualizer.addPlanarRegionsList(planarRegionsList, YoAppearance.Gray());
         polygonSnapperVisualizer.setSnappedPolygon(nonSnappedTransform, snapTransform);
      }

      PlanarRegionPolygonSnapperTest.assertSurfaceNormalsMatchAndSnapPreservesXFromAbove(snapTransform, planarRegionTransform);

      if (VISUALIZE)
      {
         ThreadTools.sleepForever();
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMovingAcrossStaircase()
   {
      ConvexPolygon2d originalPolygon = createRectanglePolygon(0.3, 0.15);
      RigidBodyTransform nonSnappedTransform = new RigidBodyTransform();

      PolygonSnapperVisualizer polygonSnapperVisualizer = null;
      if (VISUALIZE)
      {
         polygonSnapperVisualizer = new PolygonSnapperVisualizer(originalPolygon);
      }

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      int numberOfSteps = 5;

      double width = 0.8;
      double length = 0.4;
      double height = 0.1;

      generator.translate(length, 0.0, 0.0);
      generator.rotateEuler(new Vector3d(0.1, 0.1, 0.0));
      for (int i = 0; i < numberOfSteps; i++)
      {
         generator.addCubeReferencedAtBottomMiddle(length, width, height);
         generator.translate(length, 0.0, 0.0);
         height = height + 0.1;
      }

      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      if (polygonSnapperVisualizer != null)
      {
         polygonSnapperVisualizer.addPlanarRegionsList(planarRegionsList, YoAppearance.Gold(), YoAppearance.Purple(), YoAppearance.Brown());
      }

      for (double xTranslation = 0.0; xTranslation < 3.0; xTranslation = xTranslation + 0.1)
      {
         ConvexPolygon2d polygonToSnap = new ConvexPolygon2d(originalPolygon);
         nonSnappedTransform = new RigidBodyTransform();
         nonSnappedTransform.setTranslation(xTranslation, 0.0, 0.0);
         polygonToSnap.applyTransformAndProjectToXYPlane(nonSnappedTransform);

         RigidBodyTransform snapTransform = PlanarRegionsListPolygonSnapper.snapPolygonToPlanarRegionsList(polygonToSnap, planarRegionsList);
//         PlanarRegionPolygonSnapperTest.assertSurfaceNormalsMatchAndSnapPreservesXFromAbove(snapTransform, planarRegionTransform);

//         System.out.println(snapTransform);

         if (snapTransform != null)
         {
            int numberOfVertices = polygonToSnap.getNumberOfVertices();
            for (int vertexIndex = 0; vertexIndex < numberOfVertices; vertexIndex++)
            {
               Point2d vertex = polygonToSnap.getVertex(vertexIndex);
               Point3d snappedVertex = new Point3d(vertex.getX(), vertex.getY(), 0.0);

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

      if (VISUALIZE)
      {
         polygonSnapperVisualizer.cropBuffer();
         ThreadTools.sleepForever();
      }
   }

   private static ConvexPolygon2d createRectanglePolygon(double lengthX, double widthY)
   {
      ConvexPolygon2d convexPolygon = new ConvexPolygon2d();
      convexPolygon.addVertex(lengthX / 2.0, widthY / 2.0);
      convexPolygon.addVertex(-lengthX / 2.0, widthY / 2.0);
      convexPolygon.addVertex(-lengthX / 2.0, -widthY / 2.0);
      convexPolygon.addVertex(lengthX / 2.0, -widthY / 2.0);
      convexPolygon.update();
      return convexPolygon;
   }

   public static void main(String[] args)
   {
      String targetTests = PlanarRegionsListPolygonSnapperTest.class.getName();
      String targetClassesInSamePackage = PlanarRegionsListPolygonSnapper.class.getName();
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClassesInSamePackage);
   }
}
