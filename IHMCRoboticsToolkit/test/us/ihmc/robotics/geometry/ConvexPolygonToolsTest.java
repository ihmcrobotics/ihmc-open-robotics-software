package us.ihmc.robotics.geometry;

import org.junit.Test;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.awt.*;
import java.util.ArrayList;
import java.util.Random;

import static org.junit.Assert.assertTrue;

public class ConvexPolygonToolsTest
{
   private static final boolean PLOT_RESULTS = false;
   private static final boolean WAIT_FOR_BUTTON_PUSH = false;
   private static final double epsilon = 1e-7;

	@DeployableTestMethod(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testCombineDisjointPolygons()
   {
      Random random = new Random(1776L);

      ReferenceFrame zUpFrame = ReferenceFrame.constructARootFrame("someFrame", true, false, true);
      double xMin1 = 0.0, xMax1 = 1.0, yMin1 = 0.0, yMax1 = 1.0;
      ArrayList<FramePoint2d> points1 = ConvexPolygon2dTestHelpers.generateRandomCircularFramePoints(random, zUpFrame, xMin1, xMax1, yMin1, yMax1, 100);

      double xMin2 = 2.0, xMax2 = 3.0, yMin2 = 0.0, yMax2 = 2.0;
      ArrayList<FramePoint2d> points2 = ConvexPolygon2dTestHelpers.generateRandomCircularFramePoints(random, zUpFrame, xMin2, xMax2, yMin2, yMax2, 100);

      FrameConvexPolygon2d polygon1 = new FrameConvexPolygon2d(points1);
      FrameConvexPolygon2d polygon2 = new FrameConvexPolygon2d(points2);

      FrameConvexPolygon2dAndConnectingEdges frameConvexPolygon2dAndConnectingEdges = null;

      ArrayList<FramePoint2d> pointsThatShouldBeInCombinedPolygon = new ArrayList<FramePoint2d>();
      int numberOfPointsInCombinedPolygon = 10000;

      for (int i = 0; i < numberOfPointsInCombinedPolygon; i++)
      {
         int randomIndex = random.nextInt(points1.size());
         FramePoint2d firstPoint = points1.get(randomIndex);

         randomIndex = random.nextInt(points1.size());
         FramePoint2d secondPoint = points2.get(randomIndex);

         double alpha = random.nextDouble();
//         FramePoint2d morphedPoint = FramePoint2d.morph(firstPoint, secondPoint, alpha);
         FramePoint2d morphedPoint = new FramePoint2d();
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

      assertTrue(polygon1.isPointInside(connectingEdge1.getFirstEndPointCopy()));
      assertTrue(polygon2.isPointInside(connectingEdge1.getSecondEndPointCopy()));

      assertTrue(polygon1.isPointInside(connectingEdge2.getSecondEndPointCopy()));
      assertTrue(polygon2.isPointInside(connectingEdge2.getFirstEndPointCopy()));

      ArrayList<FramePoint2d> pointsThatShouldNotBeInOriginals = new ArrayList<FramePoint2d>();

//      pointsThatShouldNotBeInOriginals.add(FramePoint2d.morph(connectingEdge1.getFirstEndPointCopy(), connectingEdge1.getSecondEndPointCopy(), -epsilon));
//      pointsThatShouldNotBeInOriginals.add(FramePoint2d.morph(connectingEdge1.getFirstEndPointCopy(), connectingEdge1.getSecondEndPointCopy(), epsilon));
//      pointsThatShouldNotBeInOriginals.add(FramePoint2d.morph(connectingEdge1.getFirstEndPointCopy(), connectingEdge1.getSecondEndPointCopy(), 1.0 - epsilon));
//      pointsThatShouldNotBeInOriginals.add(FramePoint2d.morph(connectingEdge1.getFirstEndPointCopy(), connectingEdge1.getSecondEndPointCopy(), 1.0 + epsilon));
//      pointsThatShouldNotBeInOriginals.add(FramePoint2d.morph(connectingEdge2.getFirstEndPointCopy(), connectingEdge2.getSecondEndPointCopy(), -epsilon));
//      pointsThatShouldNotBeInOriginals.add(FramePoint2d.morph(connectingEdge2.getFirstEndPointCopy(), connectingEdge2.getSecondEndPointCopy(), epsilon));
//      pointsThatShouldNotBeInOriginals.add(FramePoint2d.morph(connectingEdge2.getFirstEndPointCopy(), connectingEdge2.getSecondEndPointCopy(), 1.0 - epsilon));
//      pointsThatShouldNotBeInOriginals.add(FramePoint2d.morph(connectingEdge2.getFirstEndPointCopy(), connectingEdge2.getSecondEndPointCopy(), 1.0 + epsilon));
      
      FramePoint2d point1 = new FramePoint2d();
      FramePoint2d point2 = new FramePoint2d();
      FramePoint2d point3 = new FramePoint2d();
      FramePoint2d point4 = new FramePoint2d();
      FramePoint2d point5 = new FramePoint2d();
      FramePoint2d point6 = new FramePoint2d();
      FramePoint2d point7 = new FramePoint2d();
      FramePoint2d point8 = new FramePoint2d();
      
      point1.interpolate(connectingEdge1.getFirstEndPointCopy(), connectingEdge1.getSecondEndPointCopy(), -epsilon);
      point2.interpolate(connectingEdge1.getFirstEndPointCopy(), connectingEdge1.getSecondEndPointCopy(), epsilon);
      point3.interpolate(connectingEdge1.getFirstEndPointCopy(), connectingEdge1.getSecondEndPointCopy(), 1.0 - epsilon);
      point4.interpolate(connectingEdge1.getFirstEndPointCopy(), connectingEdge1.getSecondEndPointCopy(), 1.0 + epsilon);
      point5.interpolate(connectingEdge1.getFirstEndPointCopy(), connectingEdge1.getSecondEndPointCopy(), -epsilon);
      point6.interpolate(connectingEdge1.getFirstEndPointCopy(), connectingEdge1.getSecondEndPointCopy(), epsilon);
      point7.interpolate(connectingEdge1.getFirstEndPointCopy(), connectingEdge1.getSecondEndPointCopy(), 1.0 - epsilon);
      point8.interpolate(connectingEdge1.getFirstEndPointCopy(), connectingEdge1.getSecondEndPointCopy(), 1.0 + epsilon);
      
      pointsThatShouldNotBeInOriginals.add(point1);
      pointsThatShouldNotBeInOriginals.add(point2);
      pointsThatShouldNotBeInOriginals.add(point3);
      pointsThatShouldNotBeInOriginals.add(point4);
      pointsThatShouldNotBeInOriginals.add(point5);
      pointsThatShouldNotBeInOriginals.add(point6);
      pointsThatShouldNotBeInOriginals.add(point7);
      pointsThatShouldNotBeInOriginals.add(point8);

      ArrayList<FramePoint2d> pointsThatAreNotInCombinedPolygon = new ArrayList<FramePoint2d>();
      int numberOfPointsToCheck = 10000;

      double xMin3 = -0.5, xMax3 = 3.5, yMin3 = -0.5, yMax3 = 2.5;

      ArrayList<FramePoint2d> pointsToCheck = ConvexPolygon2dTestHelpers.generateRandomRectangularFramePoints(random, zUpFrame, xMin3, xMax3, yMin3, yMax3,
            numberOfPointsToCheck);

      for (FramePoint2d pointToCheck : pointsToCheck)
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
