package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.random.RandomTools;

public class StringStretcher2dTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimpleExampleWithNoWaypoints()
   {
      StringStretcher2d stringStretcher2d = new StringStretcher2d();

      Point2D startPoint = new Point2D(0.0, 1.0);
      Point2D endPoint = new Point2D(1.0, 1.0);

      stringStretcher2d.setStartPoint(startPoint);
      stringStretcher2d.setEndPoint(endPoint);

      Point2D minPoint1 = new Point2D(0.5, 0.0);
      Point2D maxPoint1 = new Point2D(0.5, 2.0);

      stringStretcher2d.addMinMaxPoints(minPoint1, maxPoint1);

      Point2D worstMinViolator = stringStretcher2d.findWorstMinViolator(startPoint, endPoint);
      Point2D worstMaxViolator = stringStretcher2d.findWorstMaxViolator(startPoint, endPoint);

      assertNull(worstMinViolator);
      assertNull(worstMaxViolator);

      ArrayList<Point2D> waypoints = new ArrayList<Point2D>();
      stringStretcher2d.findWaypoints(waypoints);

      assertEquals(0, waypoints.size());

      List<Point2D> solution = stringStretcher2d.stretchString();
      assertEquals(3, solution.size());

      EuclidCoreTestTools.assertTuple2DEquals(startPoint, solution.get(0), 1e-7);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.5, 1.0), solution.get(1), 1e-7);
      EuclidCoreTestTools.assertTuple2DEquals(endPoint, solution.get(2), 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimpleExampleWithOneWaypointsNoInterpolation()
   {
      StringStretcher2d stringStretcher2d = new StringStretcher2d();

      Point2D startPoint = new Point2D(0.0, 1.0);
      Point2D endPoint = new Point2D(10.0, 1.0);

      stringStretcher2d.setStartPoint(startPoint);
      stringStretcher2d.setEndPoint(endPoint);

      Point2D minPoint = new Point2D(5.0, -1.0);
      Point2D maxPoint = new Point2D(5.0, 0.0);

      stringStretcher2d.addMinMaxPoints(minPoint, maxPoint);

      ArrayList<Point2D> waypoints = new ArrayList<Point2D>();
      stringStretcher2d.findWaypoints(waypoints);

      assertEquals(1, waypoints.size());

      assertTrue(maxPoint == waypoints.get(0));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimpleExampleWithAllWaypointsNoInterpolation()
   {
      StringStretcher2d stringStretcher2d = new StringStretcher2d();

      Point2D startPoint = new Point2D(0.0, 1.0);
      Point2D endPoint = new Point2D(10.0, 1.0);

      stringStretcher2d.setStartPoint(startPoint);
      stringStretcher2d.setEndPoint(endPoint);

      Point2D minPoint1 = new Point2D(2.0, 2.0);
      Point2D maxPoint1 = new Point2D(2.0, 3.0);

      stringStretcher2d.addMinMaxPoints(minPoint1, maxPoint1);

      Point2D minPoint2 = new Point2D(8.0, 2.0);
      Point2D maxPoint2 = new Point2D(8.0, 3.0);

      stringStretcher2d.addMinMaxPoints(minPoint2, maxPoint2);

      Point2D minPoint3 = new Point2D(5.0, -1.0);
      Point2D maxPoint3 = new Point2D(5.0, 0.0);

      stringStretcher2d.addMinMaxPoints(minPoint3, maxPoint3);

      ArrayList<Point2D> waypoints = new ArrayList<Point2D>();
      stringStretcher2d.findWaypoints(waypoints);

      assertEquals(3, waypoints.size());

      assertTrue(minPoint1 == waypoints.get(0));
      assertTrue(maxPoint3 == waypoints.get(1));
      assertTrue(minPoint2 == waypoints.get(2));


   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testStartAndEnd()
   {
      StringStretcher2d stringStretcher2d = new StringStretcher2d();

      Point2D startPoint = new Point2D(0.0, 1.0);
      Point2D endPoint = new Point2D(1.0, 1.0);

      stringStretcher2d.setStartPoint(startPoint);
      stringStretcher2d.setEndPoint(endPoint);

      List<Point2D> waypoints = stringStretcher2d.stretchString();
      assertEquals(2, waypoints.size());
      EuclidCoreTestTools.assertTuple2DEquals(startPoint, waypoints.get(0), 1e-7);
      EuclidCoreTestTools.assertTuple2DEquals(endPoint, waypoints.get(1), 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testRandomExample()
   {
      StringStretcher2d stringStretcher2d = new StringStretcher2d();

      Point2D startPoint = new Point2D(-10.0, 0.0);
      stringStretcher2d.setStartPoint(startPoint);
      Point2D endPoint = new Point2D(10.0, 0.0);
      stringStretcher2d.setEndPoint(endPoint);

      Random random = new Random(1776L);

      int numberOfPoints = 1000;

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point2D point = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Point2D otherPoint = new Point2D(point);
         otherPoint.setY(RandomTools.generateRandomDouble(random, -10.0, 10.0));

         if (point.getY() < otherPoint.getY())
         {
            stringStretcher2d.addMinMaxPoints(point, otherPoint);
         }

         else
         {
            stringStretcher2d.addMinMaxPoints(otherPoint, point);
         }

      }

      ArrayList<Point2D> waypoints = new ArrayList<Point2D>();
      stringStretcher2d.findWaypoints(waypoints);


      List<Point2D> stretchedString = stringStretcher2d.stretchString();
      assertEquals(numberOfPoints + 2, stretchedString.size());

      double previousX = Double.NEGATIVE_INFINITY;
      for (Point2D point2d : stretchedString)
      {
         double x = point2d.getX();
         if (x <= previousX)
         {
            fail();
         }
         else
         {
            previousX = x;
         }

         if ((!hasSameX(point2d, startPoint)) && (!hasSameX(point2d, endPoint)))
         {
            Point2D[] minMaxPoints = stringStretcher2d.findMinMaxPoints(x);
            assertTrue(point2d.getY() >= minMaxPoints[0].getY());
            assertTrue(point2d.getY() <= minMaxPoints[1].getY());
         }
      }
   }

   private boolean hasSameX(Point2D pointA, Point2D pointB)
   {
      return (Math.abs(pointA.getX() - pointB.getX()) < 1e-7);
   }

}
