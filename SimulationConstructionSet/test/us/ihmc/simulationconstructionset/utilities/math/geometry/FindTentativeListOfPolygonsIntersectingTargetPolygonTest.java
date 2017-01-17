package us.ihmc.simulationconstructionset.utilities.math.geometry;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.ArrayList;
import java.util.Random;

import javax.swing.JFrame;
import javax.swing.JScrollPane;
import javax.vecmath.Point2d;

import org.junit.Test;

import us.ihmc.graphicsDescription.plotting.artifact.PolygonArtifact;
import us.ihmc.plotting.PlotterPanel;
import us.ihmc.simulationconstructionset.util.ground.steppingStones.SteppingStones;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.BoundingBox2d;
import us.ihmc.robotics.geometry.BoundingBoxKDTree2D;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.ConvexPolygon2dIntersectionSetCalculator;
import us.ihmc.robotics.geometry.ConvexPolygonTools;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2006</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class FindTentativeListOfPolygonsIntersectingTargetPolygonTest
{
   private static final boolean VERBOSE = false;
   private static final boolean SHOW_PLOTTER = false;

	@ContinuousIntegrationTest(estimatedDuration = 0.5)
	@Test(timeout=300000)
   public void testFindTentativeListOfPolygonsIntersectingTargetPolygon()
   {

      PlotterPanel plotterpanel = createPlotterPanel();

      Random random = new Random(1776L);
      ArrayList<ConvexPolygon2d> convexPolygon2ds = new ArrayList<ConvexPolygon2d>();
      ArrayList<ConvexPolygon2d> intersectingPolygon2ds = new ArrayList<ConvexPolygon2d>();
      ArrayList<ConvexPolygon2d> steppintStonesIntersectingCaptureRegion = new ArrayList<ConvexPolygon2d>();

      ArrayList<PolygonArtifact> polygon1 = new ArrayList<PolygonArtifact>();

      int numberOfPolygons = 200;
      for (int i = 0; i < numberOfPolygons; i++)
      {
         Point2d randomPoint1 = new Point2d(generateRandomDouble(random, -100.0, 200.0), generateRandomDouble(random, -100.0, 100.0));
         Point2d randomPoint2 = new Point2d(generateRandomDouble(random, randomPoint1.getX(), 200.0), generateRandomDouble(random, randomPoint1.getY(), 100.0));
         ArrayList<Point2d> points = generateRandomCircularPoints(randomPoint1.getX(), randomPoint2.getX(), randomPoint1.getY(), randomPoint2.getY(), 10);
         ConvexPolygon2d polygon = new ConvexPolygon2d(points);
         convexPolygon2ds.add(polygon);

         polygon1.add(new PolygonArtifact("polygon" + i, false, Color.BLACK, polygon));
      }

      ArrayList<Point2d> points = generateRandomCircularPoints(-50.0, 50.0, -50.0, 50.0, 7);
      ConvexPolygon2d captureRegionPolygon = new ConvexPolygon2d(points);

      polygon1.add(new PolygonArtifact("captureRegionPolygon", false, Color.red, captureRegionPolygon));

      ConvexPolygon2dIntersectionSetCalculator convexPolygon2dIntersectionSetCalculator = new ConvexPolygon2dIntersectionSetCalculator(convexPolygon2ds);
      steppintStonesIntersectingCaptureRegion = convexPolygon2dIntersectionSetCalculator.findIntersectionPolygonList(captureRegionPolygon);

      if ((steppintStonesIntersectingCaptureRegion != null) && (steppintStonesIntersectingCaptureRegion.isEmpty() == false))
      {
         for (int i = 0; i < steppintStonesIntersectingCaptureRegion.size(); i++)
         {
            polygon1.add(new PolygonArtifact("steppintStonesIntersectingCaptureRegion" + i, true, Color.green, steppintStonesIntersectingCaptureRegion.get(i)));
         }
      }

      displayPolygons(plotterpanel, polygon1);

      int count = 0;
      for (int i = 0; i < convexPolygon2ds.size(); i++)
      {
         ConvexPolygon2d intersection = new ConvexPolygon2d();
         boolean success = ConvexPolygonTools.computeIntersectionOfPolygons(convexPolygon2ds.get(i), captureRegionPolygon, intersection);
         if (success)
         {
            intersectingPolygon2ds.add(intersection);
            count++;
         }
         else
         {
            intersectingPolygon2ds.add(null);
         }
      }

      if (count != steppintStonesIntersectingCaptureRegion.size())
         throw new RuntimeException("Bug Alert! Not equal number of intersections");

      ArrayList<ConvexPolygon2d> steppingtStonesIntersectingCaptureRegionCopy = new ArrayList<ConvexPolygon2d>(steppintStonesIntersectingCaptureRegion);
      for (int i = 0; i < convexPolygon2ds.size(); i++)
      {
         if (intersectingPolygon2ds.get(i) != null)
         {
            boolean matchingPolygonFound = false;
            for (int j = 0; j < steppingtStonesIntersectingCaptureRegionCopy.size(); j++)
            {
               if (intersectingPolygon2ds.get(i).epsilonEquals(steppingtStonesIntersectingCaptureRegionCopy.get(j), 1e-7))
               {
                  matchingPolygonFound = true;
                  steppingtStonesIntersectingCaptureRegionCopy.remove(j);

                  break;
               }
            }

            if (!matchingPolygonFound)
               throw new RuntimeException("Bug Alert! matching Polygon not found");
         }
      }

      if (!steppingtStonesIntersectingCaptureRegionCopy.isEmpty())
         throw new RuntimeException("Bug Alert! Same intersections not found");

      try
      {
         Thread.sleep(500);
      }
      catch (Exception ex)
      {
         ex.printStackTrace();
      }

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.5)
	@Test(timeout=300000)
   public void testFindTentativeListOfPolygonsIntersectingTargetPolygonTwo()
   {
      PlotterPanel plotterpanel = createPlotterPanel();

      ArrayList<ConvexPolygon2d> convexPolygon2ds = new ArrayList<ConvexPolygon2d>();
      ArrayList<ConvexPolygon2d> steppintStonesIntersectingCaptureRegion = new ArrayList<ConvexPolygon2d>();

      ArrayList<PolygonArtifact> polygon1 = new ArrayList<PolygonArtifact>();

      ArrayList<Point2d> points1 = new ArrayList<Point2d>();
      ArrayList<Point2d> points2 = new ArrayList<Point2d>();

      points1.add(new Point2d(-6.0, -2.0));
      points1.add(new Point2d(-6.0, 2.0));
      points1.add(new Point2d(6.0, 2.0));
      points1.add(new Point2d(6.0, -2.0));
      ConvexPolygon2d convexPolygon2d = new ConvexPolygon2d(points1);

      polygon1.add(new PolygonArtifact("polygon" + 1, false, Color.BLACK, points1));
      convexPolygon2ds.add(convexPolygon2d);

      points2.add(new Point2d(-5.0, -5.0));
      points2.add(new Point2d(-5.0, 5.0));
      points2.add(new Point2d(5.0, 5.0));
      points2.add(new Point2d(5.0, -5.0));

      ConvexPolygon2d captureRegionPolygon = new ConvexPolygon2d(points2);

      polygon1.add(new PolygonArtifact("captureRegionPolygon", false, Color.red, points2));

      ConvexPolygon2dIntersectionSetCalculator convexPolygon2dIntersectionSetCalculator = new ConvexPolygon2dIntersectionSetCalculator(convexPolygon2ds);
      steppintStonesIntersectingCaptureRegion = convexPolygon2dIntersectionSetCalculator.findIntersectionPolygonList(captureRegionPolygon);

      ConvexPolygon2d correctAnswer = new ConvexPolygon2d(new double[][] { { -5.0, 2.0 }, { 5.0, 2.0 }, { 5.0, -2.0 }, { -5.0, -2.0 } });

      if (!steppintStonesIntersectingCaptureRegion.get(0).epsilonEquals(correctAnswer, 1e-7))
      {
         throw new RuntimeException("Bug Alert!");
      }

      for (int i = 0; i < steppintStonesIntersectingCaptureRegion.size(); i++)
      {
         polygon1.add(new PolygonArtifact("steppintStonesIntersectingCaptureRegion" + i, true, Color.green, steppintStonesIntersectingCaptureRegion.get(i)));
      }

      displayPolygons(plotterpanel, polygon1);
      try
      {
         Thread.sleep(500);
      }
      catch (Exception ex)
      {
         ex.printStackTrace();
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.5)
	@Test(timeout=300000)
   public void testFindTentativeListOfPolygonsIntersectingTargetPolygonThree()
   {
      PlotterPanel plotterpanel = createPlotterPanel();

      ArrayList<ConvexPolygon2d> convexPolygon2ds = new ArrayList<ConvexPolygon2d>();
      ArrayList<ConvexPolygon2d> steppintStonesIntersectingCaptureRegion = new ArrayList<ConvexPolygon2d>();
      ArrayList<ConvexPolygon2d> intersectingPolygon2ds = new ArrayList<ConvexPolygon2d>();
      ArrayList<PolygonArtifact> polygon1 = new ArrayList<PolygonArtifact>();

      ArrayList<Point2d> points = new ArrayList<Point2d>();
      points.add(new Point2d(-0.5, -0.5));
      points.add(new Point2d(-0.5, 0.5));
      points.add(new Point2d(0.5, 0.5));
      points.add(new Point2d(0.5, -0.5));
      ConvexPolygon2d convexPolygon2d = new ConvexPolygon2d(points);

      SteppingStones steppingStones = new SteppingStones();
      steppingStones = SteppingStones.generateRectangularUniformSteppingStones(-15.0, -10.0, 2.0, 2.0, 0.5, 0.5, -0.1, 0.0, 9, 13, convexPolygon2d, false);

      for (int i = 0; i < 117; i++)
      {
         convexPolygon2ds.add(steppingStones.getConvexPolygons().get(i));
         polygon1.add(new PolygonArtifact("polygon" + i, false, Color.BLACK, steppingStones.getConvexPolygons().get(i)));
      }

      points.clear();
      points.add(new Point2d(-5.0, -5.0));
      points.add(new Point2d(-5.0, 5.0));
      points.add(new Point2d(5.0, 5.0));
      points.add(new Point2d(5.0, -5.0));

      ConvexPolygon2d captureRegionPolygon = new ConvexPolygon2d(points);
      polygon1.add(new PolygonArtifact("captureRegionPolygon", false, Color.blue, points));

      ConvexPolygon2dIntersectionSetCalculator convexPolygon2dIntersectionSetCalculator = new ConvexPolygon2dIntersectionSetCalculator(convexPolygon2ds);
      steppintStonesIntersectingCaptureRegion = convexPolygon2dIntersectionSetCalculator.findIntersectionPolygonList(captureRegionPolygon);

      for (int i = 0; i < steppintStonesIntersectingCaptureRegion.size(); i++)
      {
         ConvexPolygon2d convexPolygon2d2 = steppintStonesIntersectingCaptureRegion.get(i);
         polygon1.add(new PolygonArtifact("steppintStonesIntersectingCaptureRegion" + i, true, Color.green, convexPolygon2d2));
      }

      displayPolygons(plotterpanel, polygon1);

      int count = 0;
      for (int i = 0; i < convexPolygon2ds.size(); i++)
      {
         ConvexPolygon2d intersection = new ConvexPolygon2d();
         boolean success = ConvexPolygonTools.computeIntersectionOfPolygons(convexPolygon2ds.get(i), captureRegionPolygon, intersection);
         if (success)
         {
            intersectingPolygon2ds.add(intersection);
            count++;
         }
         else
         {
            intersectingPolygon2ds.add(null);
         }
      }

      if (count != steppintStonesIntersectingCaptureRegion.size())
         throw new RuntimeException("Bug Alert! Not equal number of intersections");

      ArrayList<ConvexPolygon2d> steppintStonesIntersectingCaptureRegionCopy = new ArrayList<ConvexPolygon2d>(steppintStonesIntersectingCaptureRegion);
      for (int i = 0; i < convexPolygon2ds.size(); i++)
      {
         if (intersectingPolygon2ds.get(i) != null)
         {
            boolean matchingPolygonFound = false;
            for (int j = 0; j < steppintStonesIntersectingCaptureRegionCopy.size(); j++)
            {
               if (intersectingPolygon2ds.get(i).epsilonEquals(steppintStonesIntersectingCaptureRegionCopy.get(j), 1e-7))
               {
                  matchingPolygonFound = true;
                  steppintStonesIntersectingCaptureRegionCopy.remove(j);
                  break;
               }
            }
            if (!matchingPolygonFound)
               throw new RuntimeException("Bug Alert! matching Polygon not found");
         }
      }
      if (!steppintStonesIntersectingCaptureRegionCopy.isEmpty())
         throw new RuntimeException("Bug Alert! Same intersections not found");

      try
      {
         Thread.sleep(500);
      }
      catch (Exception ex)
      {
         ex.printStackTrace();
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 1.4)
	@Test(timeout=300000)
   public void testFindTentativeListOfPolygonsIntersectingTargetPolygonTiming()
   {
      Random random = new Random(1776L);

      ArrayList<ConvexPolygon2d> convexPolygon2ds = new ArrayList<ConvexPolygon2d>();
      ArrayList<ConvexPolygon2d> captureRegionPolygon = new ArrayList<ConvexPolygon2d>();
      ArrayList<BoundingBox2d> intersectingBoundingBoxes = new ArrayList<BoundingBox2d>();
      ArrayList<BoundingBox2d> boundingBoxes = new ArrayList<BoundingBox2d>();
      ArrayList<ConvexPolygon2d> tentativeListofSteppintStonesIntersectingCaptureRegion = new ArrayList<ConvexPolygon2d>();
      ArrayList<ConvexPolygon2d> steppintStonesIntersectingCaptureRegion = new ArrayList<ConvexPolygon2d>();

      for (int i = 0; i < 200; i++)
      {
         Point2d randomPoint1 = new Point2d(generateRandomDouble(random, -200.0, 200.0), generateRandomDouble(random, -200.0, 200.0));
         Point2d randomPoint2 = new Point2d(generateRandomDouble(random, randomPoint1.getX(), 200.0), generateRandomDouble(random, randomPoint1.getY(), 200.0));
         ArrayList<Point2d> points = generateRandomCircularPoints(randomPoint1.getX(), randomPoint2.getX(), randomPoint1.getY(), randomPoint2.getY(), 15);
         ConvexPolygon2d polygon = new ConvexPolygon2d(points);
         convexPolygon2ds.add(polygon);
      }

      for (int i = 0; i < convexPolygon2ds.size(); i++)
      {
         ConvexPolygon2d convexPolygon2d = convexPolygon2ds.get(i);
         boundingBoxes.add(convexPolygon2d.getBoundingBox());
      }

      ArrayList<Object> convexPolygon2dObjects = new ArrayList<Object>(convexPolygon2ds);
      long startTime = System.currentTimeMillis();
      BoundingBoxKDTree2D kdTree = new BoundingBoxKDTree2D(boundingBoxes, convexPolygon2dObjects);
      long endTime = System.currentTimeMillis();
      double timeToBuildKDTree = endTime - startTime;
      if (VERBOSE)
         System.out.println("timeToBuildKDTreeDirectly = " + timeToBuildKDTree + " milliseconds");

      startTime = System.currentTimeMillis();
      ConvexPolygon2dIntersectionSetCalculator convexPolygon2dIntersectionSetCalculator = new ConvexPolygon2dIntersectionSetCalculator(convexPolygon2ds);
      endTime = System.currentTimeMillis();
      timeToBuildKDTree = endTime - startTime;
      if (VERBOSE)
         System.out.println("timeToBuildKDTree = " + timeToBuildKDTree + " milliseconds");

      int numTests = 10000;
      for (int i = 0; i < numTests; i++)
      {
         Point2d randomPoint1 = new Point2d(generateRandomDouble(random, -50.0, 50.0), generateRandomDouble(random, -50.0, 50.0));
         Point2d randomPoint2 = new Point2d(generateRandomDouble(random, randomPoint1.getX(), 50.0), generateRandomDouble(random, randomPoint1.getY(), 50.0));
         ArrayList<Point2d> points = generateRandomCircularPoints(randomPoint1.getX(), randomPoint2.getX(), randomPoint1.getY(), randomPoint2.getY(), 10);
         captureRegionPolygon.add(new ConvexPolygon2d(points));
      }

      startTime = System.currentTimeMillis();

      for (int i = 0; i < numTests; i++)
      {
         intersectingBoundingBoxes.clear();
         ConvexPolygon2d convexPolygon2d = captureRegionPolygon.get(i);
         intersectingBoundingBoxes = kdTree.getIntersectingBoundingBoxes(convexPolygon2d.getBoundingBox());
      }

      endTime = System.currentTimeMillis();
      double timePerBoundingBoxSearch = (endTime - startTime) / ((double) numTests);
      if (VERBOSE)
         System.out.println("timePerBoundingBoxSearch = " + timePerBoundingBoxSearch + " milliseconds per test");

      startTime = System.currentTimeMillis();

      for (int i = 0; i < numTests; i++)
      {
         tentativeListofSteppintStonesIntersectingCaptureRegion.clear();
         tentativeListofSteppintStonesIntersectingCaptureRegion = convexPolygon2dIntersectionSetCalculator
               .findTentativeListOfPolygonsIntersectingTargetPolygon(captureRegionPolygon.get(i));
      }

      endTime = System.currentTimeMillis();
      double timePerTentativeList = (endTime - startTime) / ((double) numTests);
      if (VERBOSE)
         System.out.println("timePerTentativeList = " + timePerTentativeList + " milliseconds per test");

      startTime = System.currentTimeMillis();

      for (int i = 0; i < numTests; i++)
      {
         //       if(i%10 == 0)
         //          System.out.println(i);
         steppintStonesIntersectingCaptureRegion.clear();
         steppintStonesIntersectingCaptureRegion = convexPolygon2dIntersectionSetCalculator.findIntersectionPolygonList(captureRegionPolygon.get(i));
      }

      endTime = System.currentTimeMillis();
      double timePerActualIntersectingPolygonsList = (endTime - startTime) / ((double) numTests);
      if (VERBOSE)
         System.out.println("timePerActualIntersectingPolygonsList = " + timePerActualIntersectingPolygonsList + " milliseconds per test");

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testBadCase()
   {
      ArrayList<Point2d> points = new ArrayList<Point2d>();
      points.add(new Point2d(47.213206532573494, 33.6934385384089));
      points.add(new Point2d(47.184403978806806, 33.6901486028859));
      points.add(new Point2d(47.18629896969355, 33.945259199418715));
      points.add(new Point2d(47.1996820005182, 37.28788526977694));
      points.add(new Point2d(47.20659834223872, 36.93933424308558));

      new ConvexPolygon2d(points);
   }

   private ArrayList<Point2d> generateRandomCircularPoints(double xMin, double xMax, double yMin, double yMax, int numberOfPoints)
   {
      ArrayList<Point2d> points = new ArrayList<Point2d>();

      Random random = new Random(1972L);

      Point2d zeroPoint = new Point2d((xMax + xMin) / 2.0, (yMax + yMin) / 2.0);

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point2d randomPoint = new Point2d(generateRandomDouble(random, xMin, xMax), generateRandomDouble(random, yMin, yMax));

         if (randomPoint.distance(zeroPoint) > (Math.max((xMax - xMin) / 2.0, (yMax - yMin) / 2.0)))
            continue;

         points.add(randomPoint);
      }

      return points;
   }

   private double generateRandomDouble(Random random, double min, double max)
   {
      return min + random.nextDouble() * (max - min);
   }

   private void displayPolygons(PlotterPanel plotterpanel, ArrayList<PolygonArtifact> polygonArtifacts)
   {
      if (plotterpanel == null)
         return;

      for (int i = 0; i < polygonArtifacts.size(); i++)
      {
         plotterpanel.getPlotter().addArtifact(polygonArtifacts.get(i));
         try
         {
            Thread.sleep(100);
         }
         catch (Exception ex)
         {
            ex.printStackTrace();
         }
      }
   }

   private PlotterPanel createPlotterPanel()
   {
      if (SHOW_PLOTTER)
      {
         PlotterPanel plotterpanel = new PlotterPanel();
         JFrame f = new JFrame("Plotter Panel");
         f.addWindowListener(new WindowAdapter()
         {
            public void windowClosing(WindowEvent e)
            {
               System.exit(0);
            }
         });
         f.getContentPane().add(new JScrollPane(plotterpanel), BorderLayout.CENTER);
         f.pack();
         f.setVisible(true);

         return plotterpanel;
      }

      return null;
   }
}
