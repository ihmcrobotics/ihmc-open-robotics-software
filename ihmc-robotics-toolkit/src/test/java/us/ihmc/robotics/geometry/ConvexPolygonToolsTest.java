package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.After;
import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DBasics;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class ConvexPolygonToolsTest
{
   private static final boolean PLOT_RESULTS = false;
   private static final boolean WAIT_FOR_BUTTON_PUSH = false;
   private static final double epsilon = 1e-7;

   @After
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testCombineDisjointPolygons()
   {
      Random random = new Random(1776L);

      ReferenceFrame zUpFrame = ReferenceFrame.constructARootFrame("someFrame");
      double xMin1 = 0.0, xMax1 = 1.0, yMin1 = 0.0, yMax1 = 1.0;
      ArrayList<FramePoint2D> points1 = ConvexPolygon2dTestHelpers.generateRandomCircularFramePoints(random, zUpFrame, xMin1, xMax1, yMin1, yMax1, 100);

      double xMin2 = 2.0, xMax2 = 3.0, yMin2 = 0.0, yMax2 = 2.0;
      ArrayList<FramePoint2D> points2 = ConvexPolygon2dTestHelpers.generateRandomCircularFramePoints(random, zUpFrame, xMin2, xMax2, yMin2, yMax2, 100);

      FrameConvexPolygon2D polygon1 = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(points1));
      FrameConvexPolygon2D polygon2 = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(points2));

      ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();
      FrameConvexPolygon2D combinedPolygon = new FrameConvexPolygon2D(zUpFrame);
      FrameLineSegment2D connectingEdge1 = new FrameLineSegment2D(zUpFrame);
      FrameLineSegment2D connectingEdge2 = new FrameLineSegment2D(zUpFrame);

      ArrayList<FramePoint2D> pointsThatShouldBeInCombinedPolygon = new ArrayList<FramePoint2D>();
      int numberOfPointsInCombinedPolygon = 10000;

      for (int i = 0; i < numberOfPointsInCombinedPolygon; i++)
      {
         int randomIndex = random.nextInt(points1.size());
         FramePoint2D firstPoint = points1.get(randomIndex);

         randomIndex = random.nextInt(points1.size());
         FramePoint2D secondPoint = points2.get(randomIndex);

         double alpha = random.nextDouble();
         //         FramePoint2d morphedPoint = FramePoint2d.morph(firstPoint, secondPoint, alpha);
         FramePoint2D morphedPoint = new FramePoint2D(zUpFrame);
         morphedPoint.interpolate(firstPoint, secondPoint, alpha);

         pointsThatShouldBeInCombinedPolygon.add(morphedPoint);
      }

      int numTests = 1000;

      long startTime = System.currentTimeMillis();
      for (int i = 0; i < numTests; i++)
      {
         convexPolygonTools.combineDisjointPolygons(polygon1, polygon2, combinedPolygon, connectingEdge1, connectingEdge2);
      }

      long endTime = System.currentTimeMillis();
      double timePer = (endTime - startTime) / ((double) numTests);

      PrintTools.info("timePer = " + timePer + " milliseconds per test using combineDisjointPolygons.");
      assertTrue(timePer < 1.5);

      startTime = System.currentTimeMillis();

      for (int i = 0; i < numTests; i++)
      {
         @SuppressWarnings("unused")
         FrameConvexPolygon2D expectedCombinedPolygon = new FrameConvexPolygon2D(polygon1, polygon2);
      }

      endTime = System.currentTimeMillis();
      timePer = (endTime - startTime) / ((double) numTests);

      PrintTools.info("timePer = " + timePer + " milliseconds per test using combineWith.");
      assertTrue(timePer < 2.0);

      assertTrue(polygon1.isPointInside(connectingEdge1.getFirstEndpoint()));
      assertTrue(polygon2.isPointInside(connectingEdge1.getSecondEndpoint()));

      assertTrue(polygon1.isPointInside(connectingEdge2.getSecondEndpoint()));
      assertTrue(polygon2.isPointInside(connectingEdge2.getFirstEndpoint()));

      ArrayList<FramePoint2D> pointsThatShouldNotBeInOriginals = new ArrayList<FramePoint2D>();

      //      pointsThatShouldNotBeInOriginals.add(FramePoint2d.morph(connectingEdge1.getFirstEndPointCopy(), connectingEdge1.getSecondEndPointCopy(), -epsilon));
      //      pointsThatShouldNotBeInOriginals.add(FramePoint2d.morph(connectingEdge1.getFirstEndPointCopy(), connectingEdge1.getSecondEndPointCopy(), epsilon));
      //      pointsThatShouldNotBeInOriginals.add(FramePoint2d.morph(connectingEdge1.getFirstEndPointCopy(), connectingEdge1.getSecondEndPointCopy(), 1.0 - epsilon));
      //      pointsThatShouldNotBeInOriginals.add(FramePoint2d.morph(connectingEdge1.getFirstEndPointCopy(), connectingEdge1.getSecondEndPointCopy(), 1.0 + epsilon));
      //      pointsThatShouldNotBeInOriginals.add(FramePoint2d.morph(connectingEdge2.getFirstEndPointCopy(), connectingEdge2.getSecondEndPointCopy(), -epsilon));
      //      pointsThatShouldNotBeInOriginals.add(FramePoint2d.morph(connectingEdge2.getFirstEndPointCopy(), connectingEdge2.getSecondEndPointCopy(), epsilon));
      //      pointsThatShouldNotBeInOriginals.add(FramePoint2d.morph(connectingEdge2.getFirstEndPointCopy(), connectingEdge2.getSecondEndPointCopy(), 1.0 - epsilon));
      //      pointsThatShouldNotBeInOriginals.add(FramePoint2d.morph(connectingEdge2.getFirstEndPointCopy(), connectingEdge2.getSecondEndPointCopy(), 1.0 + epsilon));

      FramePoint2D point1 = new FramePoint2D(zUpFrame);
      FramePoint2D point2 = new FramePoint2D(zUpFrame);
      FramePoint2D point3 = new FramePoint2D(zUpFrame);
      FramePoint2D point4 = new FramePoint2D(zUpFrame);
      FramePoint2D point5 = new FramePoint2D(zUpFrame);
      FramePoint2D point6 = new FramePoint2D(zUpFrame);
      FramePoint2D point7 = new FramePoint2D(zUpFrame);
      FramePoint2D point8 = new FramePoint2D(zUpFrame);

      point1.interpolate(connectingEdge1.getFirstEndpoint(), connectingEdge1.getSecondEndpoint(), -epsilon);
      point2.interpolate(connectingEdge1.getFirstEndpoint(), connectingEdge1.getSecondEndpoint(), epsilon);
      point3.interpolate(connectingEdge1.getFirstEndpoint(), connectingEdge1.getSecondEndpoint(), 1.0 - epsilon);
      point4.interpolate(connectingEdge1.getFirstEndpoint(), connectingEdge1.getSecondEndpoint(), 1.0 + epsilon);
      point5.interpolate(connectingEdge1.getFirstEndpoint(), connectingEdge1.getSecondEndpoint(), -epsilon);
      point6.interpolate(connectingEdge1.getFirstEndpoint(), connectingEdge1.getSecondEndpoint(), epsilon);
      point7.interpolate(connectingEdge1.getFirstEndpoint(), connectingEdge1.getSecondEndpoint(), 1.0 - epsilon);
      point8.interpolate(connectingEdge1.getFirstEndpoint(), connectingEdge1.getSecondEndpoint(), 1.0 + epsilon);

      pointsThatShouldNotBeInOriginals.add(point1);
      pointsThatShouldNotBeInOriginals.add(point2);
      pointsThatShouldNotBeInOriginals.add(point3);
      pointsThatShouldNotBeInOriginals.add(point4);
      pointsThatShouldNotBeInOriginals.add(point5);
      pointsThatShouldNotBeInOriginals.add(point6);
      pointsThatShouldNotBeInOriginals.add(point7);
      pointsThatShouldNotBeInOriginals.add(point8);

      ArrayList<FramePoint2D> pointsThatAreNotInCombinedPolygon = new ArrayList<FramePoint2D>();
      int numberOfPointsToCheck = 10000;

      double xMin3 = -0.5, xMax3 = 3.5, yMin3 = -0.5, yMax3 = 2.5;

      ArrayList<FramePoint2D> pointsToCheck = ConvexPolygon2dTestHelpers.generateRandomRectangularFramePoints(random, zUpFrame, xMin3, xMax3, yMin3, yMax3,
                                                                                                              numberOfPointsToCheck);

      for (FramePoint2D pointToCheck : pointsToCheck)
      {
         if (!combinedPolygon.isPointInside(pointToCheck))
         {
            pointsThatAreNotInCombinedPolygon.add(pointToCheck);
         }
      }

      if (PLOT_RESULTS)
      {
         FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(xMin1, xMax2, yMin1, yMax2);
         FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();

         plotter.addFramePoints2d(points1, Color.GREEN);
         plotter.addFramePoints2d(points2, Color.BLUE);

         plotter.addFramePoints2d(pointsThatShouldBeInCombinedPolygon, Color.YELLOW);
         plotter.addFramePoints2d(pointsThatAreNotInCombinedPolygon, Color.RED);

         plotter.addFramePoints2d(pointsThatShouldNotBeInOriginals, Color.RED);

         plotter.addFrameLineSegment2d(connectingEdge1, Color.GREEN);
         plotter.addFrameLineSegment2d(connectingEdge2, Color.RED);

         plotter.addPolygon(polygon1, Color.YELLOW);
         plotter.addPolygon(polygon2, Color.BLUE);

         plotter.addPolygon(combinedPolygon, Color.GREEN);

         waitForButtonOrPause(testFrame);
      }

      ConvexPolygon2dTestHelpers.verifyPointsAreClockwise(combinedPolygon);
      ConvexPolygon2dTestHelpers.verifyPointsAreInside(combinedPolygon, points1, 0.0);
      ConvexPolygon2dTestHelpers.verifyPointsAreInside(combinedPolygon, points2, 0.0);
      ConvexPolygon2dTestHelpers.verifyPointsAreInside(combinedPolygon, pointsThatShouldBeInCombinedPolygon, 1e-14);

      ConvexPolygon2dTestHelpers.verifyPointsAreNotInside(polygon1, pointsThatAreNotInCombinedPolygon, 0.0);
      ConvexPolygon2dTestHelpers.verifyPointsAreNotInside(polygon2, pointsThatAreNotInCombinedPolygon, 0.0);

      ConvexPolygon2dTestHelpers.verifyPointsAreNotInside(polygon1, pointsThatShouldNotBeInOriginals, 0.0);
      ConvexPolygon2dTestHelpers.verifyPointsAreNotInside(polygon2, pointsThatShouldNotBeInOriginals, 0.0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCombineDisjointPolygons2() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < 1000; i++)
      {
         List<Point2D> pointList = EuclidGeometryRandomTools.nextPointCloud2D(random, 0.0, 1.0, 100);

         Point2D offset1 = new Point2D(-1.0, 0.0);
         Point2D offset2 = new Point2D(1.0, 0.0);

         ConvexPolygon2D polygon1 = new ConvexPolygon2D();
         ConvexPolygon2D polygon2 = new ConvexPolygon2D();

         for (int index = 0; index < pointList.size(); index++)
         {
            Point2D vertex1 = new Point2D();
            vertex1.add(pointList.get(index), offset1);
            polygon1.addVertex(vertex1);

            Point2D vertex2 = new Point2D();
            vertex2.add(pointList.get(index), offset2);
            polygon2.addVertex(vertex2);
         }

         polygon1.update();
         polygon2.update();

         ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();
         FrameConvexPolygon2D actualPolygon = new FrameConvexPolygon2D();
         FrameLineSegment2D connectingEdge1 = new FrameLineSegment2D();
         FrameLineSegment2D connectingEdge2 = new FrameLineSegment2D();
         convexPolygonTools.combineDisjointPolygons(polygon1, polygon2, actualPolygon, connectingEdge1, connectingEdge2);

         ConvexPolygon2D expectedPolygon = new ConvexPolygon2D(polygon1, polygon2);
         assertTrue("Iteration: " + i + ", expected\n" + expectedPolygon + "\nactual\n" + actualPolygon, expectedPolygon.epsilonEquals(actualPolygon, epsilon));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLimitVerticesConservative()
   {
      Random random = new Random(123821L);
      int tests = 100;

      //      int increase = 0;
      //      int decrease = 0;

      for (int test = 0; test < tests; test++)
      {
         FrameConvexPolygon2D polygon = new FrameConvexPolygon2D();
         int n = random.nextInt(30) + 1;
         for (int i = 0; i < n; i++)
         {
            double x = random.nextDouble();
            double y = random.nextDouble();
            polygon.addVertex(new Point2D(x, y));
         }
         polygon.update();

         int desiredNumberOfVertices = random.nextInt(10);
         FrameConvexPolygon2D originalPolygon = new FrameConvexPolygon2D(polygon);

         //         if (desiredNumberOfVertices > polygon.getNumberOfVertices()) increase++;
         //         if (desiredNumberOfVertices < polygon.getNumberOfVertices()) decrease++;

         ConvexPolygonTools.limitVerticesConservative(polygon, desiredNumberOfVertices);

         if (PLOT_RESULTS)
         {
            FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(-0.1, 1.1, -0.1, 1.1);
            FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
            plotter.setDrawPointsLarge();

            plotter.addPolygon(originalPolygon, Color.BLUE);
            plotter.addPolygon(polygon, Color.RED);

            for (int i = 0; i < originalPolygon.getNumberOfVertices(); i++)
            {
               plotter.addFramePoint2d(new FramePoint2D(ReferenceFrame.getWorldFrame(), originalPolygon.getVertex(i)), Color.BLUE);
            }
            for (int i = 0; i < polygon.getNumberOfVertices(); i++)
            {
               plotter.addFramePoint2d(new FramePoint2D(ReferenceFrame.getWorldFrame(), polygon.getVertex(i)), Color.RED);
            }

            System.out.println("Expecting " + desiredNumberOfVertices + " Vertices.");
            waitForButtonOrPause(testFrame);
            testFrame.dispose();
         }

         // check if the number of vertices is correct
         Assert.assertTrue(desiredNumberOfVertices >= polygon.getNumberOfVertices());
         // check if the new polygon is contained in the old one
         Assert.assertTrue(ConvexPolygon2dCalculator.isPolygonInside(polygon, 10E-10, originalPolygon));
      }

      //      System.out.println("Tested " + increase + " point increases");
      //      System.out.println("Tested " + decrease + " point decreases");
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsNegativeAngle()
   {
      assertPolygons(new double[] {0, 5, 2, -2, 2, 0}, new double[] {2.5, 1, 2.8, 1, 3, .9, 4, 0, 3, -1}, new double[] {2, 0, 46.0 / 17, 6.0 / 34}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsThirdQuadrant()
   {
      assertPolygons(new double[] {-2, -1, -1, -1, -1, -2}, new double[] {-2, -2, -2, -3, -4, -4, -4, -2}, new double[] {-1.5, -1.5, -2, -2}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsNegativeAngleAndTwoVisibleVerticesOnPolygon1()
   {
      assertPolygons(new double[] {0, 0, 1, 2, 1, 0}, new double[] {2, 2, 0, 3, -1, 4}, new double[] {1, 2, 1.2, 2.4}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsParalellEdges()
   {
      assertPolygons(new double[] {0, 0, 0, 1, 1, 0, 2, 1, 1, 2}, new double[] {0, 3, 2, 3, -1, 4, 3, 4}, new double[] {1, 2, 1, 3}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsMultiplePossibleAnswers()
   {
      assertPolygons(new double[] {0, 0, 0, 1, 1, 0, 2, 1, 1, 2}, new double[] {3, 2, 2, 3, 2, 4, 4, 2}, new double[] {1, 2, 2, 3}, .001);
   }

   @Ignore
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsTwoVisiblePoints()
   {
      assertPolygons(new double[] {0, 0, 0, 1, 1, 0, 2, 1, 1, 2}, new double[] {4, 1, 1, 4, 2, 4, 4, 2}, new double[] {2, 1, 3, 2}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsTwoVisiblePoints2()
   {
      assertPolygons(new double[] {0, 0, 0, 1, 1, 0, 2, 1, 1, 2}, new double[] {4, 1, 1.5, 4, 2, 4, 4, 2}, new double[] {2, 1, 194.0 / 61, 121.0 / 61}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsOneOfTheAnglesIsZero()
   {
      assertPolygons(new double[] {0, 0, 0, 1, 1, 0, 2, 1, 1, 2}, new double[] {0, 2, 0, 3, 1, 3, .8, 2}, new double[] {.9, 1.9, .8, 2}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsTriangles()
   {
      assertPolygons(new double[] {0, 1, 1, 0, 2, 0}, new double[] {0, 3, 4, 3, 1, 2}, new double[] {.4, .8, 1, 2}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testDistanceBetweenPolygonsSharedPoint()
   {
      assertPolygons(new double[] {0, 0, 0, 1, 1, 0, 2, 1, 1, 2}, new double[] {0, 2, 0, 3, 1, 3, 1, 2}, new double[] {1, 2, 1, 2}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsPointOnEdge()
   {
      assertPolygons(new double[] {0, 0, 0, 1, 1, 0, 2, 1, 1, 2}, new double[] {0, 2, 0, 3, 1, 3, .5, 1.5}, new double[] {.5, 1.5, .5, 1.5}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsNegativeAngle2()
   {
      assertPolygons(new double[] {0, 0, 0, 1, 1, 0, 2, 1, 1, 2}, new double[] {0, 2, 0, 3, 1, 3, .4, 1.5}, new double[] {.45, 1.45, .4, 1.5}, .001);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsSolutionIsTwoVertices()
   {
      assertPolygons(new double[] {0, 0, 2, 0, 2, 2}, new double[] {4, 3, 6, 3, 6, 7}, new double[] {2, 2, 4, 3}, 0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistanceBetweenPolygonsIntersectingPolygons()
   {
      ConvexPolygon2D polygon1 = getPolygon(new double[] {0, 0, 0, 1, 1, 0, 2, 1, 1, 2});
      ConvexPolygon2D polygon2 = getPolygon(new double[] {1, 1, 0, 3, 2, 2, 3, 0});

      try
      {
         new ConvexPolygonTools().computeMinimumDistancePoints(polygon1, polygon2, new Point2D(), new Point2D());
         fail();
      }

      catch (RuntimeException re)
      {
         assertEquals(re.getMessage(), "Cannot compute minimum distance between intersecting polygons.");
      }

      try
      {
         new ConvexPolygonTools().computeMinimumDistancePoints(polygon2, polygon1, new Point2D(), new Point2D());
         fail();
      }

      catch (RuntimeException re)
      {
         assertEquals(re.getMessage(), "Cannot compute minimum distance between intersecting polygons.");
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAllMethodsForPolygonWithOnePoint()
   {
      int numberOfTrials = 100;
      double epsilon = 1e-7;
      Random random = new Random(756483920L);

      ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

      for (int i = 0; i < numberOfTrials; i++)
      {
         ArrayList<Point2D> points = new ArrayList<Point2D>();
         Point2D pointThatDefinesThePolygon = new Point2D(random.nextDouble(), random.nextDouble());
         points.add(pointThatDefinesThePolygon);
         ConvexPolygon2D polygonWithOnePoint = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(points));
         points.clear();
         Point2D pointThatDefinesAnotherPolygon = new Point2D(random.nextDouble(), random.nextDouble());
         points.add(pointThatDefinesAnotherPolygon);
         ConvexPolygon2D anotherPolygonWithOnePoint = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(points));
         points.clear();
         points.add(new Point2D(random.nextDouble(), random.nextDouble()));
         points.add(new Point2D(random.nextDouble(), random.nextDouble()));
         points.add(new Point2D(random.nextDouble(), random.nextDouble()));
         ConvexPolygon2D sparePolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(points));
         Point2D arbitraryPoint0 = new Point2D(random.nextDouble(), random.nextDouble());
         Point2D arbitraryPoint1 = new Point2D(random.nextDouble(), random.nextDouble());
         Line2D arbitraryLine = new Line2D(arbitraryPoint0, arbitraryPoint1);
         LineSegment2D arbitraryLineSegment = new LineSegment2D(arbitraryPoint0, arbitraryPoint1);

         ConvexPolygon2D intersectionPolygon = new ConvexPolygon2D();

         assertEquals(pointThatDefinesThePolygon.distance(arbitraryPoint0), polygonWithOnePoint.getClosestVertexCopy(arbitraryPoint0).distance(arbitraryPoint0),
                      epsilon);
         assertEquals(0.0, polygonWithOnePoint.getArea(), epsilon);
         assertTrue(polygonWithOnePoint.getBoundingBox().getMaxPoint().equals(pointThatDefinesThePolygon));
         assertTrue(polygonWithOnePoint.getBoundingBox().getMinPoint().equals(pointThatDefinesThePolygon));
         assertTrue(polygonWithOnePoint.getCentroid().equals(pointThatDefinesThePolygon));
         assertEquals(1, polygonWithOnePoint.getNumberOfVertices());
         assertTrue(polygonWithOnePoint.getVertex(0).equals(pointThatDefinesThePolygon));
         assertTrue(polygonWithOnePoint.getClosestEdgeCopy(arbitraryPoint0) == null);
         assertTrue(polygonWithOnePoint.getClosestEdgeIndex(arbitraryPoint0) == -1);
         assertTrue(polygonWithOnePoint.getClosestVertexCopy(arbitraryLine).equals(pointThatDefinesThePolygon));
         assertTrue(polygonWithOnePoint.getClosestVertexCopy(arbitraryPoint0).equals(pointThatDefinesThePolygon));
         assertEquals(1, polygonWithOnePoint.getNumberOfVertices());
         assertTrue(polygonWithOnePoint.getVertexCCW(0).equals(pointThatDefinesThePolygon));
         assertTrue(ConvexPolygon2dCalculator.getIntersectingEdgesCopy(arbitraryLine, polygonWithOnePoint) == null);
         assertTrue(polygonWithOnePoint.getVertex(polygonWithOnePoint.lineOfSightStartIndex(arbitraryPoint0)).equals(pointThatDefinesThePolygon));
         assertTrue(polygonWithOnePoint.getVertex(polygonWithOnePoint.lineOfSightEndIndex(arbitraryPoint0)).equals(pointThatDefinesThePolygon));
         assertTrue(polygonWithOnePoint.getCentroid().equals(pointThatDefinesThePolygon));
         assertEquals(1, polygonWithOnePoint.getNumberOfVertices());
         assertEquals(1, polygonWithOnePoint.getNumberOfVertices());
         assertTrue(polygonWithOnePoint.getVertex(0).equals(pointThatDefinesThePolygon));

         assertFalse(convexPolygonTools.computeIntersectionOfPolygons(polygonWithOnePoint, sparePolygon, intersectionPolygon));
         assertTrue(polygonWithOnePoint.intersectionWith(arbitraryLine) == null);
         assertFalse(polygonWithOnePoint.isPointInside(arbitraryPoint0));
         assertFalse(ConvexPolygon2dCalculator.isPolygonInside(sparePolygon, polygonWithOnePoint));
         assertTrue(polygonWithOnePoint.orthogonalProjectionCopy(arbitraryPoint0).equals(pointThatDefinesThePolygon));
         assertTrue(polygonWithOnePoint.pointIsOnPerimeter(pointThatDefinesThePolygon));
         assertFalse(polygonWithOnePoint.pointIsOnPerimeter(arbitraryPoint0));

         ConvexPolygon2DBasics polygonTranslation = polygonWithOnePoint.translateCopy(arbitraryPoint0);
         assertEquals(1, polygonTranslation.getNumberOfVertices());
         Point2D pointTranslation = new Point2D(pointThatDefinesThePolygon);
         pointTranslation.add(arbitraryPoint0);
         assertEquals(polygonTranslation.getVertex(0), pointTranslation);

         ConvexPolygon2D combinedPolygons = new ConvexPolygon2D(polygonWithOnePoint, anotherPolygonWithOnePoint);
         assertEquals(2, combinedPolygons.getNumberOfVertices());
         Point2DReadOnly point0 = combinedPolygons.getVertex(0);
         Point2DReadOnly point1 = combinedPolygons.getVertex(1);
         assertEqualsInEitherOrder(pointThatDefinesThePolygon, pointThatDefinesAnotherPolygon, point0, point1);

         ConvexPolygon2D combinedPolygon = new ConvexPolygon2D();
         LineSegment2D connectingEdge1 = new LineSegment2D();
         LineSegment2D connectingEdge2 = new LineSegment2D();
         convexPolygonTools.combineDisjointPolygons(polygonWithOnePoint, anotherPolygonWithOnePoint, combinedPolygon, connectingEdge1, connectingEdge2);
         assertEquals(2, combinedPolygon.getNumberOfVertices());
         point0 = combinedPolygon.getVertex(0);
         point1 = combinedPolygon.getVertex(1);
         assertEqualsInEitherOrder(pointThatDefinesThePolygon, pointThatDefinesAnotherPolygon, point0, point1);

         assertTrue(convexPolygonTools.computeIntersectionOfPolygons(polygonWithOnePoint, anotherPolygonWithOnePoint, new ConvexPolygon2D()) == false);
         ConvexPolygon2D intersection = new ConvexPolygon2D();
         convexPolygonTools.computeIntersectionOfPolygons(polygonWithOnePoint, polygonWithOnePoint, intersection);
         assertEquals(1, intersection.getNumberOfVertices());
         convexPolygonTools.computeIntersectionOfPolygons(polygonWithOnePoint, polygonWithOnePoint, intersection);
         assertTrue(intersection.getVertex(0).equals(pointThatDefinesThePolygon));
         assertTrue(polygonWithOnePoint.intersectionWith(arbitraryLineSegment) == null);
         assertTrue(polygonWithOnePoint.intersectionWith(new LineSegment2D(pointThatDefinesThePolygon, arbitraryPoint0))[0].equals(pointThatDefinesThePolygon));

         ConvexPolygonScaler shrinker = new ConvexPolygonScaler();
         ConvexPolygon2D shrunkenOnePointPolygon = new ConvexPolygon2D();

         shrinker.scaleConvexPolygon(polygonWithOnePoint, random.nextDouble(), shrunkenOnePointPolygon);

         assertTrue(shrunkenOnePointPolygon.epsilonEquals(polygonWithOnePoint, 1e-7));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAllMethodsForPolygonWithTwoPoints()
   {
      int numberOfTrials = 100;
      double epsilon = 1e-7;
      Random random = new Random(756483920L);

      ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

      for (int i = 0; i < numberOfTrials; i++)
      {
         ArrayList<Point2D> points = new ArrayList<Point2D>();
         Point2D pointThatDefinesThePolygon0 = new Point2D(random.nextDouble(), random.nextDouble());
         Point2D pointThatDefinesThePolygon1 = new Point2D(random.nextDouble(), random.nextDouble());
         LineSegment2D lineSegmentThatDefinesThePolygon = new LineSegment2D(pointThatDefinesThePolygon0, pointThatDefinesThePolygon1);
         points.add(pointThatDefinesThePolygon0);
         points.add(pointThatDefinesThePolygon1);
         ConvexPolygon2D polygonWithTwoPoints = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(points));
         points.clear();
         Point2D pointThatDefinesAnotherPolygon = new Point2D(random.nextDouble(), random.nextDouble());
         points.add(pointThatDefinesAnotherPolygon);
         ConvexPolygon2D polygonWithOnePointx = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(points));
         points.clear();
         points.add(new Point2D(random.nextDouble(), random.nextDouble()));
         points.add(new Point2D(random.nextDouble(), random.nextDouble()));
         points.add(new Point2D(random.nextDouble(), random.nextDouble()));
         ConvexPolygon2D sparePolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(points));
         Point2D arbitraryPoint0 = new Point2D(random.nextDouble(), random.nextDouble());
         Point2D arbitraryPoint1 = new Point2D(random.nextDouble(), random.nextDouble());
         Line2D arbitraryLine = new Line2D(arbitraryPoint0, arbitraryPoint1);
         LineSegment2D arbitraryLineSegment = new LineSegment2D(arbitraryPoint0, arbitraryPoint1);

         // one line tests
         assertEquals(Math.min(pointThatDefinesThePolygon0.distance(arbitraryPoint0), pointThatDefinesThePolygon1.distance(arbitraryPoint0)),
                      polygonWithTwoPoints.getClosestVertexCopy(arbitraryPoint0).distance(arbitraryPoint0), epsilon);
         assertEquals(0.0, polygonWithTwoPoints.getArea(), epsilon);
         Point2D minPoint = new Point2D(Math.min(pointThatDefinesThePolygon0.getX(), pointThatDefinesThePolygon1.getX()),
                                        Math.min(pointThatDefinesThePolygon0.getY(), pointThatDefinesThePolygon1.getY()));
         Point2D maxPoint = new Point2D(Math.max(pointThatDefinesThePolygon0.getX(), pointThatDefinesThePolygon1.getX()),
                                        Math.max(pointThatDefinesThePolygon0.getY(), pointThatDefinesThePolygon1.getY()));
         assertTrue(polygonWithTwoPoints.getBoundingBox().getMinPoint().equals(minPoint));
         assertTrue(polygonWithTwoPoints.getBoundingBox().getMaxPoint().equals(maxPoint));
         assertTrue(polygonWithTwoPoints.getCentroid().equals(lineSegmentThatDefinesThePolygon.midpoint()));
         assertEquals(2, polygonWithTwoPoints.getNumberOfVertices());
         assertEqualsInEitherOrder(pointThatDefinesThePolygon0, pointThatDefinesThePolygon1, polygonWithTwoPoints.getVertex(0),
                                   polygonWithTwoPoints.getVertex(1));
         assertFalse(polygonWithTwoPoints.isPointInside(arbitraryPoint0));
         assertFalse(ConvexPolygon2dCalculator.isPolygonInside(sparePolygon, polygonWithTwoPoints));
         assertEquals(2, polygonWithTwoPoints.getNumberOfVertices());
         assertTrue(polygonWithTwoPoints.getCentroid().getX() == 0.5 * (pointThatDefinesThePolygon0.getX() + pointThatDefinesThePolygon1.getX()));
         assertTrue(polygonWithTwoPoints.getCentroid().getY() == 0.5 * (pointThatDefinesThePolygon0.getY() + pointThatDefinesThePolygon1.getY()));
         assertEquals(2, polygonWithTwoPoints.getNumberOfVertices());

         // getClosestEdge
         LineSegment2DBasics closestEdge = polygonWithTwoPoints.getClosestEdgeCopy(arbitraryPoint0);
         Point2DReadOnly[] closestEdgeEndpoints = {closestEdge.getFirstEndpoint(), closestEdge.getSecondEndpoint()};
         assertEqualsInEitherOrder(closestEdgeEndpoints[0], closestEdgeEndpoints[1], pointThatDefinesThePolygon0, pointThatDefinesThePolygon1);

         // getClosestEdgeVertexIndicesInClockwiseOrderedList
         int edgeIndex = polygonWithTwoPoints.getClosestEdgeIndex(arbitraryPoint0);
         assertEqualsInEitherOrder(edgeIndex, polygonWithTwoPoints.getNextVertexIndex(edgeIndex), 0, 1);

         // getCounterClockwiseOrderedListOfPointsCopy
         assertEqualsInEitherOrder(polygonWithTwoPoints.getVertexCCW(0), polygonWithTwoPoints.getVertexCCW(1), pointThatDefinesThePolygon0,
                                   pointThatDefinesThePolygon1);

         // getLineOfSightVertices
         Point2DReadOnly[] lineOfSightPoints = new Point2D[2];
         lineOfSightPoints[0] = polygonWithTwoPoints.getVertex(polygonWithTwoPoints.lineOfSightStartIndex(arbitraryPoint0));
         lineOfSightPoints[1] = polygonWithTwoPoints.getVertex(polygonWithTwoPoints.lineOfSightEndIndex(arbitraryPoint0));
         assertEqualsInEitherOrder(lineOfSightPoints[0], lineOfSightPoints[1], pointThatDefinesThePolygon0, pointThatDefinesThePolygon1);

         // orthoganolProjectionCopy
         Point2DBasics expectedProjection = lineSegmentThatDefinesThePolygon.orthogonalProjectionCopy(arbitraryPoint0);
         Point2DBasics actualProjection = polygonWithTwoPoints.orthogonalProjectionCopy(arbitraryPoint0);
         assertTrue(expectedProjection.epsilonEquals(actualProjection, epsilon));

         // getClosestVertexCopy
         Point2DBasics closestVertexToLine = polygonWithTwoPoints.getClosestVertexCopy(arbitraryLine);
         if (arbitraryLine.distance(pointThatDefinesThePolygon0) < arbitraryLine.distance(pointThatDefinesThePolygon1))
            assertEquals(closestVertexToLine, pointThatDefinesThePolygon0);
         else
            assertEquals(closestVertexToLine, pointThatDefinesThePolygon1);

         Point2DBasics closestVertexToPoint = polygonWithTwoPoints.getClosestVertexCopy(arbitraryPoint0);
         if (arbitraryPoint0.distance(pointThatDefinesThePolygon0) < arbitraryPoint0.distance(pointThatDefinesThePolygon1))
            assertEquals(closestVertexToPoint, pointThatDefinesThePolygon0);
         else
            assertEquals(closestVertexToPoint, pointThatDefinesThePolygon1);

         // getIntersectingEdges
         LineSegment2D[] intersectingEdges = ConvexPolygon2dCalculator.getIntersectingEdgesCopy(arbitraryLine, polygonWithTwoPoints);
         boolean isLineAbovePoint0 = ((pointThatDefinesThePolygon0.getX() - arbitraryLine.getPoint().getX()) * arbitraryLine.slope()
               + arbitraryLine.getPoint().getY()) >= pointThatDefinesThePolygon0.getY();
         boolean isLineAbovePoint1 = ((pointThatDefinesThePolygon1.getX() - arbitraryLine.getPoint().getX()) * arbitraryLine.slope()
               + arbitraryLine.getPoint().getY()) >= pointThatDefinesThePolygon1.getY();
         boolean lineCrossesThroughPolygon = isLineAbovePoint0 ^ isLineAbovePoint1;

         if (!lineCrossesThroughPolygon)
         {
            assertTrue(intersectingEdges == null);
         }
         else
         {
            for (int j : new int[] {0, 1})
            {
               Point2DReadOnly[] endPoints = {intersectingEdges[j].getFirstEndpoint(), intersectingEdges[j].getSecondEndpoint()};
               assertEqualsInEitherOrder(endPoints[0], endPoints[1], pointThatDefinesThePolygon0, pointThatDefinesThePolygon1);
            }
         }

         // getStartingFromLeftMostClockwiseOrderedListOfPointsCopy
         assertEquals(2, polygonWithTwoPoints.getNumberOfVertices());
         Point2D leftPoint, rightPoint;
         if (pointThatDefinesThePolygon0.getX() <= pointThatDefinesThePolygon1.getX())
         {
            leftPoint = pointThatDefinesThePolygon0;
            rightPoint = pointThatDefinesThePolygon1;
         }
         else
         {
            leftPoint = pointThatDefinesThePolygon1;
            rightPoint = pointThatDefinesThePolygon0;
         }

         assertTrue((leftPoint.getX() == polygonWithTwoPoints.getVertex(0).getX()) && (leftPoint.getY() == polygonWithTwoPoints.getVertex(0).getY()));
         assertTrue((rightPoint.getX() == polygonWithTwoPoints.getVertex(1).getX()) && (rightPoint.getY() == polygonWithTwoPoints.getVertex(1).getY()));

         // intersectionWith
         Point2DBasics[] expectedIntersectionWithSparePolygon = sparePolygon.intersectionWith(new LineSegment2D(pointThatDefinesThePolygon0,
                                                                                                                pointThatDefinesThePolygon1));
         ConvexPolygon2D actualIntersectionWithSparePolygon = new ConvexPolygon2D();
         boolean success = convexPolygonTools .computeIntersectionOfPolygons(sparePolygon, polygonWithTwoPoints, actualIntersectionWithSparePolygon);

         if (expectedIntersectionWithSparePolygon == null)
         {
            assertFalse(success);
         }
         else if (expectedIntersectionWithSparePolygon.length == 1)
         {
            assertTrue(actualIntersectionWithSparePolygon.getNumberOfVertices() == 1);
            assertTrue(expectedIntersectionWithSparePolygon[0].epsilonEquals(actualIntersectionWithSparePolygon.getVertex(0), epsilon));
         }
         else if (expectedIntersectionWithSparePolygon.length == 2)
         {
            assertTrue(actualIntersectionWithSparePolygon.getNumberOfVertices() == 2);
            assertEqualsInEitherOrder(expectedIntersectionWithSparePolygon[0], expectedIntersectionWithSparePolygon[1],
                                      actualIntersectionWithSparePolygon.getVertex(0), actualIntersectionWithSparePolygon.getVertex(1));
         }
         else
         {
            fail();
         }

         // pointIsOnPerimeter
         double randomFraction = random.nextDouble();
         Point2D scaledPoint0 = new Point2D(pointThatDefinesThePolygon0);
         Point2D scaledPoint1 = new Point2D(pointThatDefinesThePolygon1);
         scaledPoint0.scale(randomFraction);
         scaledPoint1.scale(1 - randomFraction);
         Point2D randomLinearCombination = new Point2D();
         randomLinearCombination.add(scaledPoint0, scaledPoint1);
         assertTrue(polygonWithTwoPoints.pointIsOnPerimeter(randomLinearCombination));

         // STATIC METHODS

         // translateCopy
         ConvexPolygon2DBasics polygonTranslation = polygonWithTwoPoints.translateCopy(arbitraryPoint0);
         assertEquals(2, polygonTranslation.getNumberOfVertices());
         Point2D pointTranslation0 = new Point2D(pointThatDefinesThePolygon0);
         Point2D pointTranslation1 = new Point2D(pointThatDefinesThePolygon1);
         pointTranslation0.add(arbitraryPoint0);
         pointTranslation1.add(arbitraryPoint0);
         assertEqualsInEitherOrder(polygonTranslation.getVertex(0), polygonTranslation.getVertex(1), pointTranslation0, pointTranslation1);

         // combinePolygons
         ConvexPolygon2D combinedPolygons = new ConvexPolygon2D(polygonWithTwoPoints, polygonWithOnePointx);
         assertEquals(3, combinedPolygons.getNumberOfVertices());
         Point2DReadOnly point0 = combinedPolygons.getVertex(0);
         Point2DReadOnly point1 = combinedPolygons.getVertex(1);
         Point2DReadOnly point2 = combinedPolygons.getVertex(2);
         assertEqualsInAnyOrder(point0, point1, point2, pointThatDefinesThePolygon0, pointThatDefinesThePolygon1, pointThatDefinesAnotherPolygon);

         // computeIntersectionOfPolygons
         ConvexPolygon2D polygonIntersection = new ConvexPolygon2D();
         success = convexPolygonTools.computeIntersectionOfPolygons(polygonWithTwoPoints, sparePolygon, polygonIntersection);

         if (!success)
            assertTrue(sparePolygon.intersectionWith(lineSegmentThatDefinesThePolygon) == null);
         else if (polygonIntersection.getNumberOfVertices() == 1)
            assertTrue(sparePolygon.intersectionWith(lineSegmentThatDefinesThePolygon)[0].epsilonEquals(polygonIntersection.getVertex(0), epsilon));
         else if (polygonIntersection.getNumberOfVertices() == 2)
            assertEqualsInEitherOrder(sparePolygon.intersectionWith(lineSegmentThatDefinesThePolygon)[0],
                                      sparePolygon.intersectionWith(lineSegmentThatDefinesThePolygon)[1], polygonIntersection.getVertex(0),
                                      polygonIntersection.getVertex(1));
         else
            fail();

         // intersection
         Point2DBasics[] intersection = polygonWithTwoPoints.intersectionWith(arbitraryLineSegment);
         if (intersection == null)
            assertTrue(arbitraryLineSegment.intersectionWith(lineSegmentThatDefinesThePolygon) == null);
         else if (intersection.length == 1)
            assertTrue(intersection[0].distance(arbitraryLineSegment.intersectionWith(lineSegmentThatDefinesThePolygon)) < epsilon);
         else if (intersection.length == 2)
         {
            assertTrue(intersection[0].distance(arbitraryLineSegment.intersectionWith(lineSegmentThatDefinesThePolygon)) < epsilon);
            assertTrue(intersection[1].distance(arbitraryLineSegment.intersectionWith(lineSegmentThatDefinesThePolygon)) < epsilon);
            assertFalse(intersection[0].epsilonEquals(intersection[1], epsilon));
         }
         else
            fail();

         // shrinkConstantDistanceInto
         double shrinkDistance = random.nextDouble() * lineSegmentThatDefinesThePolygon.length() / 2.0;

         ConvexPolygonScaler shrinker = new ConvexPolygonScaler();

         ConvexPolygon2D shrunkenPolygon = new ConvexPolygon2D();

         shrinker.scaleConvexPolygon(polygonWithTwoPoints, shrinkDistance, shrunkenPolygon);
         shrinkDistance = lineSegmentThatDefinesThePolygon.length() / 2.0 + random.nextDouble();

         shrinker.scaleConvexPolygon(polygonWithTwoPoints, shrinkDistance, shrunkenPolygon);

         assertTrue(shrunkenPolygon.getNumberOfVertices() == 1);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIntersectionWhenFullyInside()
   {
      ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

      ArrayList<Point2D> listOfPoints = new ArrayList<Point2D>();
      listOfPoints.add(new Point2D(0.0, 0.0));
      listOfPoints.add(new Point2D(1.0, 0.0));
      listOfPoints.add(new Point2D(0.0, 1.0));
      listOfPoints.add(new Point2D(1.0, 1.0));

      ConvexPolygon2D convexPolygon2dA = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(listOfPoints));

      listOfPoints.clear();
      listOfPoints.add(new Point2D(-1.0, -1.0));
      listOfPoints.add(new Point2D(2.0, -1.0));
      listOfPoints.add(new Point2D(-1.0, 2.0));
      listOfPoints.add(new Point2D(2.0, 2.0));

      ConvexPolygon2D convexPolygon2dB = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(listOfPoints));

      ConvexPolygon2D intersection = new ConvexPolygon2D();
      convexPolygonTools.computeIntersectionOfPolygons(convexPolygon2dA, convexPolygon2dB, intersection);
      boolean epsilonEquals = intersection.epsilonEquals(convexPolygon2dA, 1e-7);
      assertTrue(epsilonEquals);

      convexPolygonTools.computeIntersectionOfPolygons(convexPolygon2dB, convexPolygon2dA, intersection);
      epsilonEquals = intersection.epsilonEquals(convexPolygon2dA, 1e-7);
      assertTrue(epsilonEquals);

      listOfPoints.clear();
      listOfPoints.add(new Point2D(0.1904001452623111, 0.07922536690619195));
      listOfPoints.add(new Point2D(0.1923482408479345, 0.5736513188711437));
      listOfPoints.add(new Point2D(0.24837080387208538, 0.5533707067242215));
      listOfPoints.add(new Point2D(0.2560381177005394, 0.550093244819894));
      listOfPoints.add(new Point2D(0.3021057612864858, 0.5276338625408057));
      listOfPoints.add(new Point2D(0.35302325196142154, 0.49669456810449586));
      listOfPoints.add(new Point2D(0.4006211967955147, 0.4608579046936889));
      listOfPoints.add(new Point2D(0.4444302495375464, 0.42047724478458476));
      listOfPoints.add(new Point2D(0.4840184248413931, 0.3759507675720234));
      listOfPoints.add(new Point2D(0.5189953579184864, 0.3277175326673503));
      listOfPoints.add(new Point2D(0.5490161537848919, 0.27625315068916595));
      listOfPoints.add(new Point2D(0.5737847881469639, 0.2220650934377122));
      listOfPoints.add(new Point2D(0.5930570263906623, 0.16568768989757945));
      listOfPoints.add(new Point2D(0.606642831891427, 0.10767685741135981));
      listOfPoints.add(new Point2D(0.1904001452623111, 0.07922536690619195));
      convexPolygon2dA = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(listOfPoints));

      listOfPoints.clear();
      listOfPoints.add(new Point2D(-0.26792484945022277, 0.5164452162023662));
      listOfPoints.add(new Point2D(-0.21938799685279367, 0.5422255592213991));
      listOfPoints.add(new Point2D(-0.1686958167513698, 0.5634565512568254));
      listOfPoints.add(new Point2D(-0.11627362387979798, 0.5799600612101443));
      listOfPoints.add(new Point2D(-0.06256124802966133, 0.591597622242303));
      listOfPoints.add(new Point2D(-0.008009343814616467, 0.5982715935305327));
      listOfPoints.add(new Point2D(0.04692439038709253, 0.5999259794889963));
      listOfPoints.add(new Point2D(0.10177905258832422, 0.5965468995798632));
      listOfPoints.add(new Point2D(0.1560944042274756, 0.5881627047730331));
      listOfPoints.add(new Point2D(0.20941473163667895, 0.5748437396773916));
      listOfPoints.add(new Point2D(0.26129266954548536, 0.5567017523393519));
      listOfPoints.add(new Point2D(0.3112929545402855, 0.5338889566605598));
      listOfPoints.add(new Point2D(0.3589960769873979, 0.5065967553012091));
      listOfPoints.add(new Point2D(0.40400180077966186, 0.4750541337839984));
      listOfPoints.add(new Point2D(0.4459325213753508, 0.43952573927242716));
      listOfPoints.add(new Point2D(0.48443643395497327, 0.4003096601427597));
      listOfPoints.add(new Point2D(0.5191904851146687, 0.3577349249793695));
      listOfPoints.add(new Point2D(0.5499030833310595, 0.31215874197725635));
      listOfPoints.add(new Point2D(0.5763165454563693, 0.26396350191354906));
      listOfPoints.add(new Point2D(0.5982092587173592, 0.2135535698334953));
      listOfPoints.add(new Point2D(0.6153975400785948, 0.16135189236915945));
      listOfPoints.add(new Point2D(0.6277371773697216, 0.10779644915591552));
      listOfPoints.add(new Point2D(0.6351246392464617, 0.05333657811986491));
      listOfPoints.add(new Point2D(0.6374979438335908, -0.00157079453245079));
      listOfPoints.add(new Point2D(0.634837178761848, -0.056464987991108585));
      listOfPoints.add(new Point2D(0.0, 0.06));
      convexPolygon2dB = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(listOfPoints));

      convexPolygonTools.computeIntersectionOfPolygons(convexPolygon2dB, convexPolygon2dA, intersection);
      epsilonEquals = intersection.epsilonEquals(convexPolygon2dA, 1e-14);
      assertTrue(epsilonEquals);

      convexPolygonTools.computeIntersectionOfPolygons(convexPolygon2dA, convexPolygon2dB, intersection);
      epsilonEquals = intersection.epsilonEquals(convexPolygon2dA, 1e-14);
      assertTrue(epsilonEquals);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIntersectionWhenFullyInsideWithRepeatedPoint()
   {
      ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

      ArrayList<Point2D> listOfPoints = new ArrayList<Point2D>();
      listOfPoints.add(new Point2D(0.19, 0.0));
      listOfPoints.add(new Point2D(0.192, 0.6));
      listOfPoints.add(new Point2D(0.25, 0.5));
      listOfPoints.add(new Point2D(0.19, 0.0));
      ConvexPolygon2D convexPolygon2dA = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(listOfPoints));

      listOfPoints.clear();
      listOfPoints.add(new Point2D(-1.0, -1.0));
      listOfPoints.add(new Point2D(2.0, -1.0));
      listOfPoints.add(new Point2D(-1.0, 2.0));
      listOfPoints.add(new Point2D(2.0, 2.0));
      ConvexPolygon2D convexPolygon2dB = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(listOfPoints));

      ConvexPolygon2D intersection = new ConvexPolygon2D();
      convexPolygonTools.computeIntersectionOfPolygons(convexPolygon2dA, convexPolygon2dB, intersection);
      boolean epsilonEquals = intersection.epsilonEquals(convexPolygon2dA, 1e-14);
      assertTrue(epsilonEquals);

      convexPolygonTools.computeIntersectionOfPolygons(convexPolygon2dB, convexPolygon2dA, intersection);
      epsilonEquals = intersection.epsilonEquals(convexPolygon2dA, 1e-14);
      assertTrue(epsilonEquals);
   }

   @ContinuousIntegrationTest(estimatedDuration = 3.8)
   @Test(timeout = 30000)
   public void testPolygonIntersections()
   {
      ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();
      Random random = new Random(1886L);

      ReferenceFrame zUpFrame = ReferenceFrame.constructARootFrame("someFrame");

      double xMin = 0.0, xMax = 1.0, yMin = 0.0, yMax = 1.0;
      double widthMax = 0.5, heightMax = 0.5;
      int numberOfPoints = 20;
      int numberOfPolygons = 30;

      ArrayList<FrameConvexPolygon2D> randomPolygons = ConvexPolygon2dTestHelpers.generateRandomPolygons(random, zUpFrame, xMin, xMax, yMin, yMax, widthMax,
                                                                                                         heightMax, numberOfPoints, numberOfPolygons);

      FrameGeometryTestFrame testFrame = null;
      FrameGeometry2dPlotter plotter = null;

      if (PLOT_RESULTS)
      {
         testFrame = new FrameGeometryTestFrame(xMin, xMax, yMin, yMax);
         plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.setDrawPointsMedium();
      }

      // Find the matrix of intersecting polygons:
      ConvexPolygon2D[][] intersectingPolygons = new ConvexPolygon2D[randomPolygons.size()][randomPolygons.size()];

      int n = randomPolygons.size();
      for (int i = 0; i < n; i++)
      {
         for (int j = 0; j < n; j++)
         {
            FrameConvexPolygon2D polygon1 = randomPolygons.get(i);
            FrameConvexPolygon2D polygon2 = randomPolygons.get(j);

            ConvexPolygon2D convexPolygon1 = new ConvexPolygon2D(polygon1);
            ConvexPolygon2D convexPolygon2 = new ConvexPolygon2D(polygon2);

            ConvexPolygon2D intersectingPolygon = new ConvexPolygon2D();
            boolean success = convexPolygonTools.computeIntersectionOfPolygons(convexPolygon1, convexPolygon2, intersectingPolygon);
            if (!success)
               intersectingPolygon = null;
            intersectingPolygons[i][j] = intersectingPolygon;

            if ((success) && (i != j))
            {
               if (PLOT_RESULTS)
               {
                  plotter.addPolygon(new FrameConvexPolygon2D(zUpFrame, intersectingPolygon), Color.BLACK);
                  plotter.repaint();
               }
            }
         }
      }

      if (PLOT_RESULTS)
      {
         plotter.addFrameConvexPolygons(randomPolygons, Color.CYAN);
         plotter.repaint();
      }

      // Generate a bunch of points. For each one, if it is in the intersection of any intersecting polygon, make sure it is in both of the polygon's parents:
      ArrayList<FramePoint2D> testPoints = ConvexPolygon2dTestHelpers.generateRandomRectangularFramePoints(random, zUpFrame, xMin, xMax, yMin, yMax, 10000);

      for (FramePoint2D testPoint : testPoints)
      {
         boolean insideAnyIntersection = false;

         for (int i = 0; i < n; i++)
         {
            for (int j = 0; j < n; j++)
            {
               FrameConvexPolygon2D polygon1 = randomPolygons.get(i);
               FrameConvexPolygon2D polygon2 = randomPolygons.get(j);

               boolean inside1 = polygon1.isPointInside(testPoint);
               boolean inside2 = polygon2.isPointInside(testPoint);

               ConvexPolygon2D intersectionPolygon = intersectingPolygons[i][j];
               if (i == j)
               {
                  assertNotNull(intersectionPolygon);
               }

               boolean insideIntersection = ((intersectionPolygon != null) && (intersectionPolygon.isPointInside(testPoint)));
               if (insideIntersection)
               {
                  insideAnyIntersection = true;
               }

               if (inside1 && inside2)
               {
                  assertTrue("inside1 and inside2, but not inside intersection", insideIntersection);
               }

               if (insideIntersection)
               {
                  assertTrue("insideIntersection, but not inside1", inside1);
                  assertTrue("insideIntersection, but not inside2", inside2);
               }
            }
         }

         if (PLOT_RESULTS)
         {
            if (insideAnyIntersection)
            {
               plotter.addFramePoint2d(testPoint, Color.GREEN);
            }
            else
            {
               plotter.addFramePoint2d(testPoint, Color.GRAY);
            }
         }

      }

      if (PLOT_RESULTS)
      {
         plotter.repaint();
         waitForButtonOrPause(testFrame);
      }
   }

   private void assertEqualsInEitherOrder(double expected0, double expected1, double actual0, double actual1)
   {
      if (expected0 == actual0)
         assertTrue(expected1 == actual1);
      else if (expected0 == actual1)
         assertTrue(expected1 == actual0);
      else
      {
         System.out.println(expected0);
         System.out.println(expected1);
         System.out.println(actual0);
         System.out.println(actual1);
         fail("Doubles are not equal in either order.");
      }
   }

   private void assertEqualsInEitherOrder(Point2DReadOnly expected0, Point2DReadOnly expected1, Point2DReadOnly actual0, Point2DReadOnly actual1)
   {
      if (expected0.epsilonEquals(actual0, epsilon))
         assertTrue(expected1.epsilonEquals(actual1, epsilon));
      else if (expected0.epsilonEquals(actual1, epsilon))
         assertTrue(expected1.epsilonEquals(actual0, epsilon));
      else
      {
         fail("Points are not equal in either order.");
      }
   }

   private void assertEqualsInAnyOrder(Point2DReadOnly expected0, Point2DReadOnly expected1, Point2DReadOnly expected2, Point2DReadOnly actual0,
                                       Point2DReadOnly actual1, Point2DReadOnly actual2)
   {
      if (expected0.equals(actual0) && expected1.equals(actual1))
         assertTrue(expected2.equals(actual2));
      else if (expected0.equals(actual0) && expected1.equals(actual2))
         assertTrue(expected2.equals(actual1));
      else if (expected0.equals(actual1) && expected1.equals(actual0))
         assertTrue(expected2.equals(actual2));
      else if (expected0.equals(actual1) && expected1.equals(actual2))
         assertTrue(expected2.equals(actual0));
      else if (expected0.equals(actual2) && expected1.equals(actual0))
         assertTrue(expected2.equals(actual1));
      else if (expected0.equals(actual2) && expected1.equals(actual1))
         assertTrue(expected2.equals(actual0));
      else
         fail("Points are not equal in any order");
   }

   private void assertPolygons(double[] p1, double[] p2, double[] expectedSolution, double epsilon)
   {
      if (expectedSolution.length != 4)
      {
         throw new RuntimeException("Invalid input.");
      }

      ConvexPolygon2D polygon1 = getPolygon(p1);
      ConvexPolygon2D polygon2 = getPolygon(p2);
      Point2D[] closestPoints = new Point2D[] {new Point2D(), new Point2D()};
      Point2D[] closestPointsReversed = new Point2D[] {new Point2D(), new Point2D()};
      ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();
      convexPolygonTools.computeMinimumDistancePoints(polygon1, polygon2, closestPoints[0], closestPoints[1]);
      convexPolygonTools.computeMinimumDistancePoints(polygon2, polygon1, closestPointsReversed[0], closestPointsReversed[1]);
      assertEquals(closestPoints[0].distance(closestPoints[1]), closestPointsReversed[0].distance(closestPointsReversed[1]), epsilon);
      assertEquals(expectedSolution[0], closestPoints[0].getX(), epsilon);
      assertEquals(expectedSolution[1], closestPoints[0].getY(), epsilon);
      assertEquals(expectedSolution[2], closestPoints[1].getX(), epsilon);
      assertEquals(expectedSolution[3], closestPoints[1].getY(), epsilon);
   }

   private ConvexPolygon2D getPolygon(double[] polygon)
   {
      if (polygon.length % 2 != 0)
      {
         throw new RuntimeException("Invalid input.");
      }

      List<Point2D> list = new ArrayList<Point2D>();
      for (int i = 0; i < polygon.length; i += 2)
      {
         list.add(new Point2D(polygon[i], polygon[i + 1]));
      }

      return new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(list));
   }

   private void waitForButtonOrPause(FrameGeometryTestFrame testFrame)
   {
      if (WAIT_FOR_BUTTON_PUSH)
         testFrame.waitForButtonPush();
      else
         pauseOneSecond();
   }

   private void pauseOneSecond()
   {
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException ex)
      {
      }
   }
}
