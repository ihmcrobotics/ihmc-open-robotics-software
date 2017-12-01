package us.ihmc.robotics.geometry;

import static org.junit.Assert.*;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class ConvexPolygonToolsTest
{
   private static final boolean PLOT_RESULTS = false;
   private static final boolean WAIT_FOR_BUTTON_PUSH = false;
   private static final double epsilon = 1e-7;

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

      FrameConvexPolygon2d polygon1 = new FrameConvexPolygon2d(points1);
      FrameConvexPolygon2d polygon2 = new FrameConvexPolygon2d(points2);

      FrameConvexPolygon2dAndConnectingEdges frameConvexPolygon2dAndConnectingEdges = null;

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
         frameConvexPolygon2dAndConnectingEdges = ConvexPolygonTools.combineDisjointPolygons(polygon1, polygon2);
      }

      long endTime = System.currentTimeMillis();
      double timePer = (endTime - startTime) / ((double) numTests);

      System.out.println("timePer = " + timePer + " milliseconds per test using combineDisjointPolygons.");
      assertTrue(timePer < 1.5);

      startTime = System.currentTimeMillis();

      for (int i = 0; i < numTests; i++)
      {
         @SuppressWarnings("unused")
         FrameConvexPolygon2d combinedPolygon = new FrameConvexPolygon2d(polygon1, polygon2);
      }

      endTime = System.currentTimeMillis();
      timePer = (endTime - startTime) / ((double) numTests);

      System.out.println("timePer = " + timePer + " milliseconds per test using combineWith.");
      assertTrue(timePer < 2.0);

      FrameConvexPolygon2d combinedPolygon = frameConvexPolygon2dAndConnectingEdges.getFrameConvexPolygon2d();

      FrameLineSegment2d connectingEdge1 = frameConvexPolygon2dAndConnectingEdges.getConnectingEdge1();
      FrameLineSegment2d connectingEdge2 = frameConvexPolygon2dAndConnectingEdges.getConnectingEdge2();

      assertTrue(polygon1.isPointInside(connectingEdge1.getFirstEndpointCopy()));
      assertTrue(polygon2.isPointInside(connectingEdge1.getSecondEndpointCopy()));

      assertTrue(polygon1.isPointInside(connectingEdge2.getSecondEndpointCopy()));
      assertTrue(polygon2.isPointInside(connectingEdge2.getFirstEndpointCopy()));

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

      point1.interpolate(connectingEdge1.getFirstEndpointCopy(), connectingEdge1.getSecondEndpointCopy(), -epsilon);
      point2.interpolate(connectingEdge1.getFirstEndpointCopy(), connectingEdge1.getSecondEndpointCopy(), epsilon);
      point3.interpolate(connectingEdge1.getFirstEndpointCopy(), connectingEdge1.getSecondEndpointCopy(), 1.0 - epsilon);
      point4.interpolate(connectingEdge1.getFirstEndpointCopy(), connectingEdge1.getSecondEndpointCopy(), 1.0 + epsilon);
      point5.interpolate(connectingEdge1.getFirstEndpointCopy(), connectingEdge1.getSecondEndpointCopy(), -epsilon);
      point6.interpolate(connectingEdge1.getFirstEndpointCopy(), connectingEdge1.getSecondEndpointCopy(), epsilon);
      point7.interpolate(connectingEdge1.getFirstEndpointCopy(), connectingEdge1.getSecondEndpointCopy(), 1.0 - epsilon);
      point8.interpolate(connectingEdge1.getFirstEndpointCopy(), connectingEdge1.getSecondEndpointCopy(), 1.0 + epsilon);

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

   @Test(timeout = 30000)
   public void testCombineDisjointPolygons2() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < 100; i++)
      {
         List<Point2D> pointList = EuclidGeometryRandomTools.nextPointCloud2D(random, 0.0, 1.0, 100);

         Point2D offset1 = new Point2D(-1.0, 0.0);
         Point2D offset2 = new Point2D(1.0, 0.0);

         ConvexPolygon2D polygon1 = new ConvexPolygon2D();
         ConvexPolygon2D polygon2 = new ConvexPolygon2D();
         ConvexPolygon2D expectedPolygon = new ConvexPolygon2D();

         for (int index = 0; index < pointList.size(); index++)
         {
            Point2D vertex1 = new Point2D();
            vertex1.add(pointList.get(index), offset1);
            polygon1.addVertex(vertex1);

            Point2D vertex2 = new Point2D();
            vertex2.add(pointList.get(index), offset2);
            polygon2.addVertex(vertex2);

            expectedPolygon.addVertex(vertex1);
            expectedPolygon.addVertex(vertex2);
         }

         polygon1.update();
         polygon2.update();

         expectedPolygon.update();

         ConvexPolygon2D actualPolygon = ConvexPolygonTools.combineDisjointPolygons(polygon1, polygon2).getConvexPolygon2d();

         assertTrue("Iteration: " + i + ", expected\n" + expectedPolygon + "\nactual\n" + actualPolygon, expectedPolygon.epsilonEquals(actualPolygon, epsilon));
      }
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
         FrameConvexPolygon2d polygon = new FrameConvexPolygon2d();
         int n = random.nextInt(30) + 1;
         for (int i = 0; i < n; i++)
         {
            double x = random.nextDouble();
            double y = random.nextDouble();
            polygon.addVertex(new Point2D(x, y));
         }
         polygon.update();

         int desiredNumberOfVertices = random.nextInt(10);
         FrameConvexPolygon2d originalPolygon = new FrameConvexPolygon2d(polygon);

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
         Assert.assertTrue(ConvexPolygon2dCalculator.isPolygonInside(polygon.getConvexPolygon2d(), 10E-10, originalPolygon.getConvexPolygon2d()));
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
    @Test(timeout=300000)
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
         ConvexPolygonTools.computeMinimumDistancePoints(polygon1, polygon2);
         fail();
      }

      catch (RuntimeException re)
      {
         assertEquals(re.getMessage(), "Cannot compute minimum distance between intersecting polygons.");
      }

      try
      {
         ConvexPolygonTools.computeMinimumDistancePoints(polygon2, polygon1);
         fail();
      }

      catch (RuntimeException re)
      {
         assertEquals(re.getMessage(), "Cannot compute minimum distance between intersecting polygons.");
      }
   }

   private void assertPolygons(double[] p1, double[] p2, double[] expectedSolution, double epsilon)
   {
      if (expectedSolution.length != 4)
      {
         throw new RuntimeException("Invalid input.");
      }

      ConvexPolygon2D polygon1 = getPolygon(p1);
      ConvexPolygon2D polygon2 = getPolygon(p2);
      Point2DReadOnly[] closestPoints = ConvexPolygonTools.computeMinimumDistancePoints(polygon1, polygon2);
      Point2DReadOnly[] closestPointsReversed = ConvexPolygonTools.computeMinimumDistancePoints(polygon2, polygon1);
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

      return new ConvexPolygon2D(list);
   }
}
