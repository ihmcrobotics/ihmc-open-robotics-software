package us.ihmc.robotics.geometry;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.random.RandomGeometry;

public class StringStretcher2dTest
{

	@Test
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

      Point2DReadOnly worstMinViolator = stringStretcher2d.findWorstMinViolator(startPoint, endPoint);
      Point2DReadOnly worstMaxViolator = stringStretcher2d.findWorstMaxViolator(startPoint, endPoint);

      assertNull(worstMinViolator);
      assertNull(worstMaxViolator);

      List<Point2DBasics> waypoints = new ArrayList<>();
      stringStretcher2d.findWaypoints(waypoints);

      assertEquals(0, waypoints.size());
      List<Point2DBasics> solution = new ArrayList<>();
      stringStretcher2d.stretchString(solution);
      assertEquals(3, solution.size());

      EuclidCoreTestTools.assertTuple2DEquals(startPoint, solution.get(0), 1e-7);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.5, 1.0), solution.get(1), 1e-7);
      EuclidCoreTestTools.assertTuple2DEquals(endPoint, solution.get(2), 1e-7);
   }

	@Test
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

      List<Point2DBasics> waypoints = new ArrayList<>();
      stringStretcher2d.findWaypoints(waypoints);

      assertEquals(1, waypoints.size());

      assertTrue(maxPoint.epsilonEquals(waypoints.get(0), 1e-6));
   }

	@Test
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

      List<Point2DBasics> waypoints = new ArrayList<>();
      stringStretcher2d.findWaypoints(waypoints);

      assertEquals(3, waypoints.size());

      assertTrue(minPoint1.epsilonEquals(waypoints.get(0), 1e-8));
      assertTrue(maxPoint3.epsilonEquals(waypoints.get(1), 1e-8));
      assertTrue(minPoint2.epsilonEquals(waypoints.get(2), 1e-8));


   }

	@Test
   public void testStartAndEnd()
   {
      StringStretcher2d stringStretcher2d = new StringStretcher2d();

      Point2D startPoint = new Point2D(0.0, 1.0);
      Point2D endPoint = new Point2D(1.0, 1.0);

      stringStretcher2d.setStartPoint(startPoint);
      stringStretcher2d.setEndPoint(endPoint);

      List<Point2DBasics> waypoints = new ArrayList<>();
      stringStretcher2d.stretchString(waypoints);
      assertEquals(2, waypoints.size());
      EuclidCoreTestTools.assertTuple2DEquals(startPoint, waypoints.get(0), 1e-7);
      EuclidCoreTestTools.assertTuple2DEquals(endPoint, waypoints.get(1), 1e-7);
   }

	@Test
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
         Point2D point = RandomGeometry.nextPoint2D(random, 10.0, 10.0);
         Point2D otherPoint = new Point2D(point);
         otherPoint.setY(RandomNumbers.nextDouble(random, -10.0, 10.0));

         if (point.getY() < otherPoint.getY())
         {
            stringStretcher2d.addMinMaxPoints(point, otherPoint);
         }

         else
         {
            stringStretcher2d.addMinMaxPoints(otherPoint, point);
         }

      }

      List<Point2DBasics> waypoints = new ArrayList<>();
      stringStretcher2d.findWaypoints(waypoints);

      List<Point2DBasics> stretchedString = new ArrayList<>();
      stringStretcher2d.stretchString(stretchedString);
      assertEquals(numberOfPoints + 2, stretchedString.size());

      double previousX = Double.NEGATIVE_INFINITY;
      for (Point2DBasics point2d : stretchedString)
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
            MinMaxPointHolder minMaxPoints = stringStretcher2d.findMinMaxPoints(x);
            assertTrue(point2d.getY() >= minMaxPoints.getMinPoint().getY());
            assertTrue(point2d.getY() <= minMaxPoints.getMaxPoint().getY());
         }
      }
   }

   private boolean hasSameX(Point2DReadOnly pointA, Point2DReadOnly pointB)
   {
      return (Math.abs(pointA.getX() - pointB.getX()) < 1e-7);
   }

}
