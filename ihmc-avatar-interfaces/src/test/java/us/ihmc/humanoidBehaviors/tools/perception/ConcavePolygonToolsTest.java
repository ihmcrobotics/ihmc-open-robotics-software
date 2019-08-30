package us.ihmc.humanoidBehaviors.tools.perception;

import javafx.application.Platform;
import javafx.stage.Stage;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXVisualizers.RandomColorFunction;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHull;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcavePolygonTools;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.CountDownLatch;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class ConcavePolygonToolsTest
{
   public static final boolean visualize = false;

   @Test
   public void testKeepRightSideOfS()
   {
      // create simple convex polygon
      ConcaveHull concaveSPolygon = drawSPolygon();

      LogTools.info("{}", concaveSPolygon.getVertex(0));

      // create line and up direction
      Line2D yAxis = new Line2D(0.0, 0.0, 0.0, -1.0);

      // cut it above a line
      List<ConcaveHull> result = ConcavePolygonTools.cutPolygonToLeftOfLine(concaveSPolygon, yAxis);

      assertEquals(2, result.size(), "should be two polygons");

      // create the ideal result
      ConcaveHull cutPolygon1 = new ConcaveHull();
      cutPolygon1.addVertex(0.0, 2.0);
      cutPolygon1.addVertex(2.0, 2.0);
      cutPolygon1.addVertex(2.0, 1.0);
      cutPolygon1.addVertex(0.0, 1.0);

      ConcaveHull cutPolygon2 = new ConcaveHull();
      cutPolygon2.addVertex(0.0, 0.0);
      cutPolygon2.addVertex(2.0, 0.0);
      cutPolygon2.addVertex(-2.0, -3.0);
      cutPolygon2.addVertex(0.0, -3.0);
      cutPolygon2.addVertex(0.0, -2.0);
      cutPolygon2.addVertex(1.0, -2.0);
      cutPolygon2.addVertex(1.0, -1.0);
      cutPolygon2.addVertex(0.0, -1.0);

      visualizePlanarRegions(concaveSPolygon, result);

      // assert equal
      assertTrue(result.get(0).epsilonEquals(cutPolygon1, 1e-5));
   }

   @Test
   public void testKeepLeftSideOfS()
   {
      // create simple convex polygon
      ConcaveHull concaveSPolygon = drawSPolygon();

      LogTools.info("{}", concaveSPolygon.getVertex(0));

      // create line and up direction
      Line2D yAxis = new Line2D(0.0, 0.0, 0.0, 1.0);

      // cut it above a line
      List<ConcaveHull> result = ConcavePolygonTools.cutPolygonToLeftOfLine(concaveSPolygon, yAxis);

      assertEquals(2, result.size(), "should be two polygons");

      // create the ideal result
      ConcaveHull cutPolygon1 = new ConcaveHull();
      cutPolygon1.addVertex(0.0, -3.0);
      cutPolygon1.addVertex(-2.0, -3.0);
      cutPolygon1.addVertex(-2.0, -2.0);
      cutPolygon1.addVertex(0.0, -2.0);

      ConcaveHull cutPolygon2 = new ConcaveHull();
      cutPolygon2.addVertex(0.0, -1.0);
      cutPolygon2.addVertex(-2.0, -1.0);
      cutPolygon2.addVertex(-2.0, -2.0);
      cutPolygon2.addVertex(0.0, 2.0);
      cutPolygon2.addVertex(0.0, 1.0);
      cutPolygon2.addVertex(-1.0, 1.0);
      cutPolygon2.addVertex(-1.0, 0.0);
      cutPolygon2.addVertex(0.0, 0.0);

      visualizePlanarRegions(concaveSPolygon, result);

      // assert equal
      assertTrue(result.get(0).epsilonEquals(cutPolygon1, 1e-5));
   }

   private ConcaveHull drawSPolygon()
   {
      ConcaveHull concaveSPolygon = new ConcaveHull();
      concaveSPolygon.addVertex(-2.0, 2.0);
      concaveSPolygon.addVertex(2.0, 2.0);
      concaveSPolygon.addVertex(2.0, 1.0);
      concaveSPolygon.addVertex(-1.0, 1.0);
      concaveSPolygon.addVertex(-1.0, 0.0);
      concaveSPolygon.addVertex(2.0, 0.0);
      concaveSPolygon.addVertex(2.0, -3.0);
      concaveSPolygon.addVertex(-2.0, -3.0);
      concaveSPolygon.addVertex(-2.0, -2.0);
      concaveSPolygon.addVertex(1.0, -2.0);
      concaveSPolygon.addVertex(1.0, -1.0);
      concaveSPolygon.addVertex(-2.0, -1.0);
      return concaveSPolygon;
   }

   @Test
   public void testCutSimpleConvexPolygonAbove()
   {
      // create simple convex polygon
      ConcaveHull size2square0center = new ConcaveHull();
      size2square0center.addVertex(1.0, 1.0);
      size2square0center.addVertex(1.0, -1.0);
      size2square0center.addVertex(-1.0, -1.0);
      size2square0center.addVertex(-1.0, 1.0);

      LogTools.info("{}", size2square0center.getVertex(0));

      // create line and up direction
      Line2D yAxis = new Line2D(0.0, 0.0, 0.0, -1.0);

      // cut it above a line
      List<ConcaveHull> result = ConcavePolygonTools.cutPolygonToLeftOfLine(size2square0center, yAxis);

      visualizePlanarRegions(size2square0center, result);

      assertEquals(1, result.size(), "supposed to cut");

      // create the ideal result
      ConcaveHull aboveYAxisRectangle = new ConcaveHull();
      aboveYAxisRectangle.addVertex(1.0, 1.0);
      aboveYAxisRectangle.addVertex(1.0, -1.0);
      aboveYAxisRectangle.addVertex(0.0, -1.0);
      aboveYAxisRectangle.addVertex(0.0, 1.0);

      // assert equal
      assertTrue(result.get(0).epsilonEquals(aboveYAxisRectangle, 1e-5));
   }

   @Test
   public void testCutSimpleConvexPolygonBelow()
   {
      // create simple convex polygon
      ConcaveHull size2square0center = new ConcaveHull();
      size2square0center.addVertex(1.0, 1.0);
      size2square0center.addVertex(1.0, -1.0);
      size2square0center.addVertex(-1.0, -1.0);
      size2square0center.addVertex(-1.0, 1.0);

      // create line and up direction
      Line2D yAxis = new Line2D(0.0, 0.0, 0.0, 1.0);

      // cut it above a line
      List<ConcaveHull> result = ConcavePolygonTools.cutPolygonToLeftOfLine(size2square0center, yAxis);

      visualizePlanarRegions(size2square0center, result);

      assertEquals(1, result.size(), "supposed to cut");

      // create the ideal result
      ConcaveHull expected = new ConcaveHull();
      expected.addVertex(-1.0, 1.0);
      expected.addVertex(0.0, 1.0);
      expected.addVertex(0.0, -1.0);
      expected.addVertex(-1.0, -1.0);

      // assert equal
      assertTrue(result.get(0).epsilonEquals(expected, 1e-5));
   }

   @Test
   public void testKeepSideSimpleConvexPolygonAbove1()
   {
      // create simple convex polygon
      ConcaveHull size2square0center = new ConcaveHull();
      size2square0center.addVertex(1.0, 1.0);
      size2square0center.addVertex(1.0, -1.0);
      size2square0center.addVertex(-1.0, -1.0);
      size2square0center.addVertex(-1.0, 1.0);

      LogTools.info("{}", size2square0center.getVertex(0));

      // create line and up direction
      Line2D yAxis = new Line2D(1.0, 0.0, 0.0, -1.0);

      // cut it above a line
      List<ConcaveHull> result = ConcavePolygonTools.cutPolygonToLeftOfLine(size2square0center, yAxis);
      LogTools.info("{}", result.get(0));

      visualizePlanarRegions(size2square0center, result);

      assertEquals(1, result.size(), "supposed to cut");

      // create the ideal result
      ConcaveHull expected = new ConcaveHull();
      expected.addVertex(0.9999999999999999, 1.0);
      expected.addVertex(1.00001, 1.0);
      expected.addVertex(1.00001, -1.0);
      expected.addVertex(1.0, -0.9999999999999999);

      // assert equal
      assertTrue(result.get(0).epsilonEquals(expected, 1e-5));
   }

   @Test
   public void testKeepLongSideSimpleConvexPolygonAbove1() // this is a pretty crazy edge case where the initial polygon has a redundant point along
   {                                                       // a colinear intersection with the cutting line
      // create simple convex polygon
      ConcaveHull size2square0center = new ConcaveHull();
      size2square0center.addVertex(1.0, 1.0);
      size2square0center.addVertex(1.0, 0.0);
      size2square0center.addVertex(1.0, -1.0);
      size2square0center.addVertex(-1.0, -1.0);

      LogTools.info("{}", size2square0center);

      // create line and up direction
      Line2D yAxis = new Line2D(1.0, 0.0, 0.0, -1.0);

      // cut it above a line
      List<ConcaveHull> result = ConcavePolygonTools.cutPolygonToLeftOfLine(size2square0center, yAxis);
      LogTools.info("{}", result.get(0));

      visualizePlanarRegions(size2square0center, result);

      LogTools.info("{}", result.get(0));
      assertEquals(1, result.size(), "supposed to cut");

      // create the ideal result
      ConcaveHull expected = new ConcaveHull();
      expected.addVertex(1.0, 0.9999899999999999);
      expected.addVertex(1.00001, 1.0);
      expected.addVertex(1.00001, 0.0);
      expected.addVertex(1.00001, -1.0);
      expected.addVertex(1.0, -1.0);

      // assert equal
      assertTrue(result.get(0).epsilonEquals(expected, 1e-5));
   }


   @Test
   public void testRemoveAllSimpleConvexPolygonAbove2()
   {
      // create simple convex polygon
      ConcaveHull size2square0center = new ConcaveHull();
      size2square0center.addVertex(1.0, 1.0);
      size2square0center.addVertex(1.0, -1.0);
      size2square0center.addVertex(-1.0, -1.0);
      size2square0center.addVertex(-1.0, 1.0);

      LogTools.info("{}", size2square0center.getVertex(0));

      // create line and up direction
      Line2D yAxis = new Line2D(2.0, 0.0, 0.0, -1.0);

      // cut it above a line
      List<ConcaveHull> result = ConcavePolygonTools.cutPolygonToLeftOfLine(size2square0center, yAxis);

      visualizePlanarRegions(size2square0center, result);

      assertEquals(0, result.size(), "supposed to cut");
   }

   @Test
   public void testKeepAllSimpleConvexPolygonAbove1()
   {
      // create simple convex polygon
      ConcaveHull size2square0center = new ConcaveHull();
      size2square0center.addVertex(1.0, 1.0);
      size2square0center.addVertex(-1.0, 1.0);
      size2square0center.addVertex(1.0, -1.0);
      size2square0center.addVertex(-1.0, -1.0);

      LogTools.info("{}", size2square0center);

      // create line and up direction
      Line2D yAxis = new Line2D(1.0, 0.0, 0.0, 1.0);

      // cut it above a line
      List<ConcaveHull> result = ConcavePolygonTools.cutPolygonToLeftOfLine(size2square0center, yAxis);
      LogTools.info("{}", result.get(0));

      visualizePlanarRegions(size2square0center, result);

      assertEquals(1, result.size(), "supposed to cut");
      LogTools.debug("{}", result.get(0));

      // create the ideal result
      ConcaveHull expected = new ConcaveHull();
      expected.addVertex(1.0, 1.0);
      expected.addVertex(-1.0, 1.0);
      expected.addVertex(1.0, -1.0);
      expected.addVertex(-1.0, -1.0);

      // assert equal
      assertTrue(result.get(0).epsilonEquals(expected, 1e-5));
   }

   @Test
   public void testKeepAllSimpleConvexPolygonAbove2()
   {
      // create simple convex polygon
      ConcaveHull size2square0center = new ConcaveHull();
      size2square0center.addVertex(1.0, 1.0);
      size2square0center.addVertex(1.0, -1.0);
      size2square0center.addVertex(-1.0, -1.0);
      size2square0center.addVertex(-1.0, 1.0);

      LogTools.info("{}", size2square0center.getVertex(0));

      // create line and up direction
      Line2D yAxis = new Line2D(2.0, 0.0, 0.0, 1.0);

      // cut it above a line
      List<ConcaveHull> result = ConcavePolygonTools.cutPolygonToLeftOfLine(size2square0center, yAxis);

      visualizePlanarRegions(size2square0center, result);

      assertEquals(1, result.size(), "supposed to cut");

      // create the ideal result
      ConcaveHull expected = new ConcaveHull();
      expected.addVertex(1.0, 1.0);
      expected.addVertex(1.0, -1.0);
      expected.addVertex(-1.0, -1.0);
      expected.addVertex(-1.0, 1.0);

      // assert equal
      assertTrue(result.get(0).epsilonEquals(expected, 1e-5));
   }

   private void visualizePlanarRegions(ConcaveHull original, List<ConcaveHull> results)
   {
      if (!visualize)
         return;

      JavaFXApplicationCreator.createAJavaFXApplication();

      int id = 0;
      RandomColorFunction colors = new RandomColorFunction();
      ArrayList<PlanarRegionsGraphic> planarRegionGraphics = new ArrayList<PlanarRegionsGraphic>();

      ArrayList<ConcaveHull> hulls = new ArrayList<>();
      hulls.add(original);
      hulls.addAll(results);
      ArrayList<PlanarRegion> resultingRegions = new ArrayList<>();
      for (ConcaveHull concaveHull : hulls)
      {
         List<ConvexPolygon2D> decomposedPolygons = new ArrayList<>();
         ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHull, 0.10, decomposedPolygons); // TODO: tune depth threshold?

         Point2D[] concaveHullsVertices = new Point2D[concaveHull.getNumberOfVertices()];
         concaveHull.getConcaveHullVertices().toArray(concaveHullsVertices);

         PlanarRegion resultingRegion = new PlanarRegion(new RigidBodyTransform(), concaveHullsVertices, decomposedPolygons);
         resultingRegion.setRegionId(id++);
         resultingRegions.add(resultingRegion);
      }

      PlanarRegionsGraphic regionsGraphic = new PlanarRegionsGraphic(false);
      regionsGraphic.setColorFunction(colors);
      regionsGraphic.generateMeshes(new PlanarRegionsList(resultingRegions));
      regionsGraphic.update();
      planarRegionGraphics.add(regionsGraphic);

      final CountDownLatch countDownLatch = new CountDownLatch(1);

      Platform.runLater(new Runnable()
      {
         @Override
         public void run()
         {
            //            double preferredWidth = 1000.0;
            //            double preferredHeight = 1000.0;

            View3DFactory view3dFactory = new View3DFactory(800, 600);
            view3dFactory.addCameraController(0.05, 2000.0, true);
            view3dFactory.addWorldCoordinateSystem(0.3);
            view3dFactory.addDefaultLighting();

            for (PlanarRegionsGraphic regionsGraphic : planarRegionGraphics)
            {
               view3dFactory.addNodeToView(regionsGraphic);
            }

            Stage stage = new Stage();
            stage.setTitle(getClass().getSimpleName());
            stage.setMaximized(false);
            stage.setScene(view3dFactory.getScene());

            Random random = new Random(System.nanoTime());
            stage.setX(Math.abs(random.nextInt() % 800));
            stage.setY(Math.abs(random.nextInt() % 800));
            stage.show();

            countDownLatch.countDown();
         }
      });

      try
      {
         countDownLatch.await();
      }
      catch (InterruptedException e)
      {
      }

      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      // TODO: Uncomment when ihmc-commons released
//      MutationTestFacilitator mutationTestFacilitator = new MutationTestFacilitator();
//      mutationTestFacilitator.addClassesToMutate(ConcavePolygonTools.class, ConvexPolygonTools.class);
//      mutationTestFacilitator.addMethodsToMutate("cutPolygonToLeftOfLine");
//      mutationTestFacilitator.addTestClassesToRun(ConcavePolygonToolsTest.class);
//      mutationTestFacilitator.addSourceDirectories(PathTools.findPathInline(".", "ihmc-open-robotics-software", "ihmc-robotics-toolkit/src"));
//      mutationTestFacilitator.addSourceDirectories(PathTools.findPathInline(".", "ihmc-open-robotics-software", "robot-environment-awareness/src"));
//      mutationTestFacilitator.doMutationTest();
//      mutationTestFacilitator.openResultInBrowser();

      MutationTestFacilitator.facilitateMutationTestForClass(ConcavePolygonTools.class, ConvexPolygonTools.class);
   }
}
