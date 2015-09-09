package us.ihmc.robotics.geometry;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import javax.vecmath.Point2d;
import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class LineSegment2dTest
{
   private static Random ran = new Random(100L);
   private static LineSegment2d testSegment1;
   private static Point2d segment1Point1;
   private static Point2d segment1Point2;

   private static LineSegment2d testSegment2;
   private static Point2d segment2Point1;
   private static Point2d segment2Point2;

   @Before
   public void setUp() throws Exception
   {
      double x1;
      double y1;
      double x2;
      double y2;

      do
      {
         x1 = ran.nextDouble() * 500 - 250;
         y1 = ran.nextDouble() * 500 - 250;
         x2 = ran.nextDouble() * 500 - 250;
         y2 = ran.nextDouble() * 500 - 250;

      }
      while ((x1 == x2) || (y1 == y2));

      segment1Point1 = new Point2d(x1, y1);
      segment1Point2 = new Point2d(x2, y2);
      testSegment1 = new LineSegment2d(segment1Point1, segment1Point2);


      do
      {
         x1 = ran.nextDouble() * 500 - 250;
         y1 = ran.nextDouble() * 500 - 250;
         x2 = ran.nextDouble() * 500 - 250;
         y2 = ran.nextDouble() * 500 - 250;
      }
      while ((x1 == x2) || (y1 == y2));

      segment2Point1 = new Point2d(x1, y1);
      segment2Point2 = new Point2d(x2, y2);
      testSegment2 = new LineSegment2d(segment2Point1, segment2Point2);
   }

   @After
   public void tearDown() throws Exception
   {
      testSegment1 = null;
      testSegment2 = null;
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testDistancePoint2dLineSegment2d()
   {
      LineSegment2d line1 = new LineSegment2d(-10.0, 0.0, 10.0, 0.0);
      Point2d point1 = new Point2d(-10.0, 10.0);
      Point2d point2 = new Point2d(0.0, 10.0);
      Point2d point3 = new Point2d(0.0, -10.0);
      Point2d point4 = new Point2d(-10.0, 0.0);
      Point2d point5 = new Point2d(10.0, 0.0);
      Point2d point6 = new Point2d(10.5, 0.0);
      Point2d point7 = new Point2d(0.0, 1.2);
      Point2d point8 = new Point2d(10.1, 0.0);
      Point2d point9 = new Point2d(0.0, 0.0);
      double delta = 0.00001;
      assertEquals(10.0, line1.distance(point1), delta);
      assertEquals(10.0, line1.distance(point2), delta);
      assertEquals(10.0, line1.distance(point3), delta);
      assertEquals(0.0, line1.distance(point4), delta);
      assertEquals(0.0, line1.distance(point5), delta);
      assertEquals(0.5, line1.distance(point6), delta);
      assertEquals(1.2, line1.distance(point7), delta);
      assertEquals(0.1, line1.distance(point8), delta);
      assertEquals(0.0, line1.distance(point9), delta);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testLineSegment2dDoubleDoubleDoubleDouble()
   {
      boolean fail = false;

      // WORKING CASES
      for (int i = 0; i < 100; i++)
      {
         double x1 = ran.nextDouble() * 500 - 250;
         double y1 = ran.nextDouble() * 500 - 250;
         double x2 = ran.nextDouble() * 500 - 250;
         double y2 = ran.nextDouble() * 500 - 250;

         if ((x1 != x2) || (y1 != y2))
         {
            LineSegment2d test = new LineSegment2d(x1, y1, x2, y2);
            assertEquals(test.getFirstEndPointCopy().x, x1, 0.001);
            assertEquals(test.getFirstEndPointCopy().y, y1, 0.001);
            assertEquals(test.getSecondEndPointCopy().x, x2, 0.001);
            assertEquals(test.getSecondEndPointCopy().y, y2, 0.001);
         }
      }

      // SHOULD FAIL

      try
      {
         double x1 = ran.nextDouble() * 500 - 250;
         double y1 = ran.nextDouble() * 500 - 250;
         LineSegment2d test = new LineSegment2d(x1, y1, x1, y1);
      }
      catch (RuntimeException e)
      {
         fail = true;
      }

      assertTrue(fail);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testLineSegment2dPoint2dArray()
   {
      boolean fail = false;

      // WORKING CASES
      for (int i = 0; i < 100; i++)
      {
         double x1 = ran.nextDouble() * 500 - 250;
         double y1 = ran.nextDouble() * 500 - 250;
         double x2 = ran.nextDouble() * 500 - 250;
         double y2 = ran.nextDouble() * 500 - 250;

         if ((x1 != x2) || (y1 != y2))
         {
            Point2d[] points = new Point2d[2];
            points[0] = new Point2d(x1, y1);
            points[1] = new Point2d(x2, y2);
            LineSegment2d test = new LineSegment2d(points);
            assertEquals(test.getFirstEndPointCopy(), points[0]);
            assertEquals(test.getSecondEndPointCopy(), points[1]);
         }
      }

      // SHOULD FAIL

      try
      {
         double x1 = ran.nextDouble() * 500 - 250;
         double y1 = ran.nextDouble() * 500 - 250;
         Point2d[] points = new Point2d[2];
         points[0] = new Point2d(x1, y1);
         points[1] = new Point2d(x1, y1);
         new LineSegment2d(points);
      }
      catch (RuntimeException e)
      {
         fail = true;
      }

      assertTrue(fail);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testLineSegment2dPoint2dPoint2d()
   {
      boolean fail = false;

      // WORKING CASES
      for (int i = 0; i < 100; i++)
      {
         double x1 = ran.nextDouble() * 500 - 250;
         double y1 = ran.nextDouble() * 500 - 250;
         double x2 = ran.nextDouble() * 500 - 250;
         double y2 = ran.nextDouble() * 500 - 250;

         if ((x1 != x2) || (y1 != y2))
         {
            Point2d[] points = new Point2d[2];
            points[0] = new Point2d(x1, y1);
            points[1] = new Point2d(x2, y2);
            LineSegment2d test = new LineSegment2d(points[0], points[1]);
            assertEquals(test.getFirstEndPointCopy(), points[0]);
            assertEquals(test.getSecondEndPointCopy(), points[1]);
         }
      }

      // SHOULD FAIL

      try
      {
         double x1 = ran.nextDouble() * 500 - 250;
         double y1 = ran.nextDouble() * 500 - 250;
         Point2d[] points = new Point2d[2];
         points[0] = new Point2d(x1, y1);
         points[1] = new Point2d(x1, y1);
         new LineSegment2d(points[0], points[1]);
      }
      catch (RuntimeException e)
      {
         fail = true;
      }

      assertTrue(fail);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testLineSegment2dLineSegment2d()
   {
      // WORKING CASES
      for (int i = 0; i < 100; i++)
      {
         double x1 = ran.nextDouble() * 500 - 250;
         double y1 = ran.nextDouble() * 500 - 250;
         double x2 = ran.nextDouble() * 500 - 250;
         double y2 = ran.nextDouble() * 500 - 250;

         if ((x1 != x2) || (y1 != y2))
         {
            Point2d[] points = new Point2d[2];
            points[0] = new Point2d(x1, y1);
            points[1] = new Point2d(x2, y2);
            LineSegment2d test = new LineSegment2d(points[0], points[1]);
            LineSegment2d test2 = new LineSegment2d(test);
            assertEquals(test.getFirstEndPointCopy(), test2.getFirstEndPointCopy());
            assertEquals(test.getSecondEndPointCopy(), test2.getSecondEndPointCopy());
         }
      }

   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testGetEndpointsCopy()
   {
      Point2d[] pointsCopy = testSegment1.getEndpointsCopy();
      assertEquals(pointsCopy[0], testSegment1.getFirstEndPointCopy());
      assertEquals(pointsCopy[1], testSegment1.getSecondEndPointCopy());
      assertEquals(pointsCopy[0], segment1Point1);
      assertEquals(pointsCopy[1], segment1Point2);

      // make sure that chaning the copy does not change the origional
      pointsCopy[0].set(pointsCopy[0].x - 10.0, pointsCopy[0].y - 10.0);
      pointsCopy[1].set(pointsCopy[1].x - 10.0, pointsCopy[1].y - 10.0);

      assertFalse(pointsCopy[0].equals(testSegment1.getFirstEndPointCopy()));
      assertFalse(pointsCopy[1].equals(testSegment1.getSecondEndPointCopy()));
      assertFalse(pointsCopy[0].equals(segment1Point1));
      assertFalse(pointsCopy[1].equals(segment1Point2));

   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testGetEndpointsPoint2dPoint2d()
   {
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testGetEndpoints()
   {
      Point2d[] pointsCopy = testSegment1.getEndpoints();
      assertEquals(pointsCopy[0], testSegment1.getFirstEndPointCopy());
      assertEquals(pointsCopy[1], testSegment1.getSecondEndPointCopy());
      assertEquals(pointsCopy[0], segment1Point1);
      assertEquals(pointsCopy[1], segment1Point2);

      // make sure that chaning the copy does not change the origional
      pointsCopy[0].set(pointsCopy[0].x - 10.0, pointsCopy[0].y - 10.0);
      pointsCopy[1].set(pointsCopy[1].x - 10.0, pointsCopy[1].y - 10.0);

      assertEquals(pointsCopy[0], testSegment1.getFirstEndPointCopy());
      assertEquals(pointsCopy[1], testSegment1.getSecondEndPointCopy());

   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testGetFirstEndPointCopy()
   {
      Point2d pointCopy = testSegment1.getFirstEndPointCopy();
      assertEquals(pointCopy, segment1Point1);


      // make sure that chaning the copy does not change the origional
      pointCopy.set(pointCopy.x - 10.0, pointCopy.y - 10.0);


      assertFalse(pointCopy.equals(segment1Point1));


   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testGetSecondEndPointCopy()
   {
      Point2d pointCopy = testSegment1.getSecondEndPointCopy();
      assertEquals(pointCopy, segment1Point2);


      // make sure that chaning the copy does not change the origional
      pointCopy.set(pointCopy.x - 10.0, pointCopy.y - 10.0);


      assertFalse(pointCopy.equals(segment1Point2));

   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testSetPoint2dPoint2d()
   {
      Point2d[] pointsCopy = testSegment1.getEndpointsCopy();

      pointsCopy[0].set(pointsCopy[0].x - 10.0, pointsCopy[0].y - 10.0);
      pointsCopy[1].set(pointsCopy[1].x - 10.0, pointsCopy[1].y - 10.0);

      testSegment1.set(pointsCopy[0], pointsCopy[1]);

      assertEquals(pointsCopy[0], testSegment1.getFirstEndPointCopy());
      assertEquals(pointsCopy[1], testSegment1.getSecondEndPointCopy());

      boolean throwEx = false;
      try
      {
         pointsCopy[0] = pointsCopy[1];
         testSegment1.set(pointsCopy[0], pointsCopy[1]);
      }
      catch (Exception e)
      {
         throwEx = true;
      }

      assertTrue(throwEx);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testSetDoubleDoubleDoubleDouble()
   {
      Point2d[] pointsCopy = testSegment1.getEndpointsCopy();

      pointsCopy[0].set(pointsCopy[0].x - 10.0, pointsCopy[0].y - 10.0);
      pointsCopy[1].set(pointsCopy[1].x - 10.0, pointsCopy[1].y - 10.0);

      testSegment1.set(pointsCopy[0].x, pointsCopy[0].y, pointsCopy[1].x, pointsCopy[1].y);

      assertEquals(pointsCopy[0], testSegment1.getFirstEndPointCopy());
      assertEquals(pointsCopy[1], testSegment1.getSecondEndPointCopy());
      boolean throwEx = false;
      try
      {
         pointsCopy[0] = pointsCopy[1];
         testSegment1.set(pointsCopy[0].x, pointsCopy[0].y, pointsCopy[1].x, pointsCopy[1].y);
      }
      catch (Exception e)
      {
         throwEx = true;
      }

      assertTrue(throwEx);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testSetPoint2dArray()
   {
      Point2d[] pointsCopy = testSegment1.getEndpointsCopy();



      pointsCopy[0].set(pointsCopy[0].x - 10.0, pointsCopy[0].y - 10.0);
      pointsCopy[1].set(pointsCopy[1].x - 10.0, pointsCopy[1].y - 10.0);

      testSegment1.set(pointsCopy);

      assertEquals(pointsCopy[0], testSegment1.getFirstEndPointCopy());
      assertEquals(pointsCopy[1], testSegment1.getSecondEndPointCopy());
      boolean throwEx = false;
      try
      {
         pointsCopy[0] = pointsCopy[1];
         testSegment1.set(pointsCopy);
      }
      catch (Exception e)
      {
         throwEx = true;
      }

      assertTrue(throwEx);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testSetLineSegment2d()
   {
      testSegment1.set(testSegment2);

      assertEquals(testSegment2.getFirstEndPointCopy(), testSegment1.getFirstEndPointCopy());
      assertEquals(testSegment2.getSecondEndPointCopy(), testSegment1.getSecondEndPointCopy());
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testFlipDirection()
   {
      testSegment1.flipDirection();
      assertEquals(segment1Point2, testSegment1.getFirstEndPointCopy());
      assertEquals(segment1Point1, testSegment1.getSecondEndPointCopy());

   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testMidpoint()
   {
      Point2d midPoint = testSegment1.midpoint();
      assertEquals(midPoint.distance(testSegment1.getFirstEndPointCopy()), midPoint.distance(testSegment1.getSecondEndPointCopy()), 0.001);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testLength()
   {
      assertEquals(segment1Point1.distance(segment1Point2), testSegment1.length(), 0.001);
      testSegment1.flipDirection();
      assertEquals(segment1Point1.distance(segment1Point2), testSegment1.length(), 0.001);

   }

// @Test(timeout=300000)
// public void testDotProduct()
// {
//    fail("Not yet implemented");
// }
//
// @Test(timeout=300000)
// public void testIsBetweenEndpoints()
// {
//    fail("Not yet implemented");
// }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testPercentageAlongLineSegment()
   {
      LineSegment2d line1 = new LineSegment2d(-10, 0, 10, 0);
      assertEquals(0.5, line1.percentageAlongLineSegment(new Point2d(0.0, 0.0)), 0.001);
      assertEquals(0.0, line1.percentageAlongLineSegment(new Point2d(-10.0, 0.0)), 0.001);
      assertEquals(1.0, line1.percentageAlongLineSegment(new Point2d(10.0, 0.0)), 0.001);
      assertEquals(0.5, line1.percentageAlongLineSegment(new Point2d(0.0, 5.0)), 0.001);

   }

// @Test(timeout=300000)
// public void testOrthogonalProjection()
// {
//    fail("Not yet implemented");
// }
//
// @Test(timeout=300000)
// public void testOrthogonalProjectionCopy()
// {
//    fail("Not yet implemented");
// }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testIntersectionWithLineSegment2d()
   {
      LineSegment2d line1 = new LineSegment2d(-10.0, 0.0, 10.0, 0.0);
      LineSegment2d line2 = new LineSegment2d(-10.0, 10.0, 10.0, 0.0);
      LineSegment2d line3 = new LineSegment2d(0.0, 10.0, 0.0, -10.0);
      LineSegment2d line4 = new LineSegment2d(0.0, -10.0, 0.0, 10.0);
      LineSegment2d line5 = new LineSegment2d(-10.0, 0.0, 10.0, 0.0);
      LineSegment2d line6 = new LineSegment2d(10.0, 0.0, -10.0, 0.0);
      LineSegment2d line7 = new LineSegment2d(10.0, 0.0, 20.0, 0.0);
      LineSegment2d line8 = new LineSegment2d(10.0, 0.0, -20.0, 0.0);
      LineSegment2d line9 = new LineSegment2d(10.1, 0.0, 20.0, 0.0);



      assertEquals(null, line1.intersectionWith(line1));
      assertEquals(null, line1.intersectionWith(line5));
      assertEquals(null, line1.intersectionWith(line6));
      assertEquals(new Point2d(10.0, 0.0), line1.intersectionWith(line2));
      assertEquals(new Point2d(0.0, 0.0), line1.intersectionWith(line3));
      assertEquals(new Point2d(0.0, 0.0), line1.intersectionWith(line4));


      assertEquals(new Point2d(10.0, 0.0), line1.intersectionWith(line7));
      assertEquals(null, line1.intersectionWith(line8));
      assertEquals(null, line1.intersectionWith(line9));




   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testIntersectionWithLine2d()
   {
      LineSegment2d line1 = new LineSegment2d(-10.0, 0.0, 10.0, 0.0);
      Line2d line2 = new Line2d(new Point2d(-10.0, 10.0), new Point2d(10.0, 0.0));
      Line2d line3 = new Line2d(new Point2d(0.0, 10.0), new Point2d(0.0, -10.0));
      Line2d line4 = new Line2d(new Point2d(0.0, -10.0), new Point2d(0.0, 10.0));
      Line2d line5 = new Line2d(new Point2d(-10.0, 0.0), new Point2d(10.0, 0.0));
      Line2d line6 = new Line2d(new Point2d(10.0, 0.0), new Point2d(-10.0, 0.0));
      Line2d line7 = new Line2d(new Point2d(10.0, 0.0), new Point2d(20.0, 0.0));
      Line2d line8 = new Line2d(new Point2d(10.0, 0.0), new Point2d(-20.0, 0.0));
      Line2d line9 = new Line2d(new Point2d(10.1, 0.0), new Point2d(20.0, 0.0));
      Line2d line10 = new Line2d(new Point2d(10.0, 0.0), new Point2d(20.0, 1.0));


      assertEquals(null, line1.intersectionWith(line5));
      assertEquals(null, line1.intersectionWith(line6));
      assertEquals(new Point2d(10.0, 0.0), line1.intersectionWith(line2));
      assertEquals(new Point2d(10.0, 0.0), line1.intersectionWith(line10));

      assertEquals(new Point2d(0.0, 0.0), line1.intersectionWith(line3));
      assertEquals(new Point2d(0.0, 0.0), line1.intersectionWith(line4));


      assertEquals(null, line1.intersectionWith(line7));
      assertEquals(null, line1.intersectionWith(line8));
      assertEquals(null, line1.intersectionWith(line9));

   }


// @Test(timeout=300000)
// public void testIntersectionWithConvexPolygon2d()
// {
//    fail("Not yet implemented");
// }
//

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testDistancePoint2d()
   {
      LineSegment2d line1 = new LineSegment2d(-10.0, 0.0, 10.0, 0.0);
      Point2d point1 = new Point2d(-10.0, 10.0);
      Point2d point2 = new Point2d(0.0, 10.0);
      Point2d point3 = new Point2d(0.0, -10.0);
      Point2d point4 = new Point2d(-10.0, 0.0);
      Point2d point5 = new Point2d(10.0, 0.0);
      Point2d point6 = new Point2d(10.5, 0.0);
      Point2d point7 = new Point2d(0.0, 1.2);
      Point2d point8 = new Point2d(10.1, 0.0);
      Point2d point9 = new Point2d(0.0, 0.0);
      double delta = 0.00001;
      assertEquals(10.0, line1.distance(point1), delta);
      assertEquals(10.0, line1.distance(point2), delta);
      assertEquals(10.0, line1.distance(point3), delta);
      assertEquals(0.0, line1.distance(point4), delta);
      assertEquals(0.0, line1.distance(point5), delta);
      assertEquals(0.5, line1.distance(point6), delta);
      assertEquals(1.2, line1.distance(point7), delta);
      assertEquals(0.1, line1.distance(point8), delta);
      assertEquals(0.0, line1.distance(point9), delta);




   }

//
// @Test(timeout=300000)
// public void testDistanceLine2d()
// {
//    fail("Not yet implemented");
// }
//
// @Test(timeout=300000)
// public void testDistanceLineSegment2d()
// {
//    fail("Not yet implemented");
// }
//
// @Test(timeout=300000)
// public void testDistanceConvexPolygon2d()
// {
//    fail("Not yet implemented");
// }
//
// @Test(timeout=300000)
// public void testToString()
// {
//    fail("Not yet implemented");
// }
//
// @Test(timeout=300000)
// public void testApplyTransformTransform3D()
// {
//    fail("Not yet implemented");
// }
//
// @Test(timeout=300000)
// public void testApplyTransformTransform3DBoolean()
// {
//    fail("Not yet implemented");
// }
//
// @Test(timeout=300000)
// public void testApplyTransformCopyTransform3D()
// {
//    fail("Not yet implemented");
// }
//
// @Test(timeout=300000)
// public void testApplyTransformCopyTransform3DBoolean()
// {
//    fail("Not yet implemented");
// }
//
// @Test(timeout=300000)
// public void testPointBetweenEndPointsGivenParameter()
// {
//    fail("Not yet implemented");
// }

   @DeployableTestMethod(duration = 0.0)
   @Test(timeout = 30000)
   public void testShiftToLeftAndRightCopy()
   {
      double distanceToShift = 0.2;
      double epsilon = 1e-7;

      // Pointing straight up:
      LineSegment2d lineSegment = new LineSegment2d(0.0, 0.0, 0.0, 1.0);
      LineSegment2d shiftedLineSegment = lineSegment.shiftToRightCopy(distanceToShift);

      Point2d firstShiftedEndpoint = shiftedLineSegment.getFirstEndPointCopy();
      Point2d secondShiftedEndpoint = shiftedLineSegment.getSecondEndPointCopy();

      assertEquals(distanceToShift, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(0.0, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(distanceToShift, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(1.0, secondShiftedEndpoint.getY(), epsilon);

      shiftedLineSegment = lineSegment.shiftToLeftCopy(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndPointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndPointCopy();

      assertEquals(-distanceToShift, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(0.0, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(-distanceToShift, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(1.0, secondShiftedEndpoint.getY(), epsilon);

      // Pointing straight along x:
      lineSegment = new LineSegment2d(0.0, 0.0, 1.0, 0.0);
      shiftedLineSegment = lineSegment.shiftToRightCopy(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndPointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndPointCopy();

      assertEquals(0.0, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(-distanceToShift, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(1.0, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(-distanceToShift, secondShiftedEndpoint.getY(), epsilon);

      shiftedLineSegment = lineSegment.shiftToLeftCopy(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndPointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndPointCopy();

      assertEquals(0.0, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(distanceToShift, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(1.0, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(distanceToShift, secondShiftedEndpoint.getY(), epsilon);

      // Pointing at (1,1)
      lineSegment = new LineSegment2d(0.0, 0.0, 1.0, 1.0);
      shiftedLineSegment = lineSegment.shiftToRightCopy(distanceToShift);
      double distanceAtFortyFiveDegrees = distanceToShift * Math.sqrt(2.0) / 2.0;

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndPointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndPointCopy();

      assertEquals(distanceAtFortyFiveDegrees, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(-distanceAtFortyFiveDegrees, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(1.0 + distanceAtFortyFiveDegrees, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(1.0 - distanceAtFortyFiveDegrees, secondShiftedEndpoint.getY(), epsilon);

      shiftedLineSegment = lineSegment.shiftToLeftCopy(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndPointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndPointCopy();

      assertEquals(-distanceAtFortyFiveDegrees, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(distanceAtFortyFiveDegrees, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(1.0 - distanceAtFortyFiveDegrees, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(1.0 + distanceAtFortyFiveDegrees, secondShiftedEndpoint.getY(), epsilon);
   }

   @DeployableTestMethod(duration = 0.0)
   @Test(timeout = 30000)
   public void testShiftToLeftAndRight()
   {
      double distanceToShift = 0.2;
      double epsilon = 1e-7;

      // Pointing straight up:
      LineSegment2d lineSegment = new LineSegment2d(0.0, 0.0, 0.0, 1.0);
      LineSegment2d shiftedLineSegment = new LineSegment2d(lineSegment);
      shiftedLineSegment.shiftToRight(distanceToShift);

      Point2d firstShiftedEndpoint = shiftedLineSegment.getFirstEndPointCopy();
      Point2d secondShiftedEndpoint = shiftedLineSegment.getSecondEndPointCopy();

      assertEquals(distanceToShift, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(0.0, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(distanceToShift, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(1.0, secondShiftedEndpoint.getY(), epsilon);

      shiftedLineSegment.set(lineSegment);
      shiftedLineSegment.shiftToLeft(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndPointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndPointCopy();

      assertEquals(-distanceToShift, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(0.0, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(-distanceToShift, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(1.0, secondShiftedEndpoint.getY(), epsilon);

      // Pointing straight along x:
      lineSegment = new LineSegment2d(0.0, 0.0, 1.0, 0.0);
      shiftedLineSegment.set(lineSegment);
      shiftedLineSegment.shiftToRight(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndPointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndPointCopy();

      assertEquals(0.0, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(-distanceToShift, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(1.0, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(-distanceToShift, secondShiftedEndpoint.getY(), epsilon);

      shiftedLineSegment.set(lineSegment);
      shiftedLineSegment.shiftToLeft(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndPointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndPointCopy();

      assertEquals(0.0, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(distanceToShift, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(1.0, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(distanceToShift, secondShiftedEndpoint.getY(), epsilon);

      // Pointing at (1,1)
      lineSegment = new LineSegment2d(0.0, 0.0, 1.0, 1.0);
      shiftedLineSegment.set(lineSegment);
      shiftedLineSegment.shiftToRight(distanceToShift);

      double distanceAtFortyFiveDegrees = distanceToShift * Math.sqrt(2.0) / 2.0;

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndPointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndPointCopy();

      assertEquals(distanceAtFortyFiveDegrees, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(-distanceAtFortyFiveDegrees, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(1.0 + distanceAtFortyFiveDegrees, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(1.0 - distanceAtFortyFiveDegrees, secondShiftedEndpoint.getY(), epsilon);

      shiftedLineSegment.set(lineSegment);
      shiftedLineSegment.shiftToLeft(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndPointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndPointCopy();

      assertEquals(-distanceAtFortyFiveDegrees, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(distanceAtFortyFiveDegrees, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(1.0 - distanceAtFortyFiveDegrees, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(1.0 + distanceAtFortyFiveDegrees, secondShiftedEndpoint.getY(), epsilon);
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testIsPointOnLeftRightSide()
   {
      LineSegment2d lineSegment = new LineSegment2d(0.0, 0.0, 0.0, 1.0);

      Point2d point = new Point2d(0.1, 0.5);
      assertFalse(lineSegment.isPointOnLeftSideOfLineSegment(point));
      assertTrue(lineSegment.isPointOnRightSideOfLineSegment(point));

      point = new Point2d(-0.1, 0.5);
      assertTrue(lineSegment.isPointOnLeftSideOfLineSegment(point));
      assertFalse(lineSegment.isPointOnRightSideOfLineSegment(point));

      lineSegment = new LineSegment2d(0.0, 2.0, 4.0, 6.0);

      point = new Point2d(0.0, 0.0);
      assertTrue(lineSegment.isPointOnRightSideOfLineSegment(point));

      point = new Point2d(4.1, 6.0);
      assertTrue(lineSegment.isPointOnRightSideOfLineSegment(point));

      point = new Point2d(3.9, 6.0);
      assertTrue(lineSegment.isPointOnLeftSideOfLineSegment(point));

   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testOrthogonalProjectionCopyPoint2dLineSegment2d()
   {
      Point2d startPoint = new Point2d(-10.0, 0.0);
      Point2d endPoint = new Point2d(10.0, 0.0);
      LineSegment2d line1 = new LineSegment2d(startPoint, endPoint);

      Point2d origionalPoint = new Point2d(-20.0, 10.0);
      Point2d projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(startPoint, projectedPoint);

      origionalPoint = new Point2d(-20.0, -10.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(startPoint, projectedPoint);

      origionalPoint = new Point2d(-20.0, 0.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(startPoint, projectedPoint);

      origionalPoint = new Point2d(20.0, 10.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(endPoint, projectedPoint);

      origionalPoint = new Point2d(20.0, -10.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(endPoint, projectedPoint);

      origionalPoint = new Point2d(20.0, 0.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(endPoint, projectedPoint);

      origionalPoint = new Point2d(0.0, 10.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(new Point2d(0.0, 0.0), projectedPoint);

      origionalPoint = new Point2d(0.0, -10.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(new Point2d(0.0, 0.0), projectedPoint);


      origionalPoint = new Point2d(5.0, 0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(new Point2d(5.0, 0.0), projectedPoint);


   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testIntersectionLine2dLineSegment2d()
   {
      LineSegment2d line1 = new LineSegment2d(-10.0, 0.0, 10.0, 0.0);
      Line2d line2 = new Line2d(new Point2d(-10.0, 10.0), new Point2d(10.0, 0.0));
      Line2d line3 = new Line2d(new Point2d(0.0, 10.0), new Point2d(0.0, -10.0));
      Line2d line4 = new Line2d(new Point2d(0.0, -10.0), new Point2d(0.0, 10.0));
      Line2d line5 = new Line2d(new Point2d(-10.0, 0.0), new Point2d(10.0, 0.0));
      Line2d line6 = new Line2d(new Point2d(10.0, 0.0), new Point2d(-10.0, 0.0));
      Line2d line7 = new Line2d(new Point2d(10.0, 0.0), new Point2d(20.0, 0.0));
      Line2d line8 = new Line2d(new Point2d(10.0, 0.0), new Point2d(-20.0, 0.0));
      Line2d line9 = new Line2d(new Point2d(10.1, 0.0), new Point2d(20.0, 0.0));
      Line2d line10 = new Line2d(new Point2d(10.0, 0.0), new Point2d(20.0, 1.0));


      assertEquals(null, line1.intersectionWith(line5));
      assertEquals(null, line1.intersectionWith(line6));
      assertEquals(new Point2d(10.0, 0.0), line1.intersectionWith(line2));
      assertEquals(new Point2d(10.0, 0.0), line1.intersectionWith(line10));

      assertEquals(new Point2d(0.0, 0.0), line1.intersectionWith(line3));
      assertEquals(new Point2d(0.0, 0.0), line1.intersectionWith(line4));


      assertEquals(null, line1.intersectionWith(line7));
      assertEquals(null, line1.intersectionWith(line8));
      assertEquals(null, line1.intersectionWith(line9));
   }

	@DeployableTestMethod(duration = 0.0)
	@Test(timeout = 30000)
   public void testIntersectionLineSegment2dLineSegment2d()
   {
      Point2d colTestp1 = new Point2d(1.5, -5);
      Point2d colTestp2 = new Point2d(1.5, 0);
      Point2d colTestp3 = new Point2d(1.5, 5);

      LineSegment2d colTestLine1 = new LineSegment2d(colTestp1, colTestp2);
      LineSegment2d colTestLine2 = new LineSegment2d(colTestp2, colTestp1);

      LineSegment2d colTestLine3 = new LineSegment2d(colTestp1, colTestp3);
      LineSegment2d colTestLine4 = new LineSegment2d(colTestp3, colTestp1);

      LineSegment2d colTestLine5 = new LineSegment2d(colTestp2, colTestp3);
      LineSegment2d colTestLine6 = new LineSegment2d(colTestp3, colTestp2);


      assertEquals(null, colTestLine1.intersectionWith(colTestLine2));
      assertEquals(null, colTestLine1.intersectionWith(colTestLine3));
      assertEquals(null, colTestLine1.intersectionWith(colTestLine4));
      assertEquals(null, colTestLine2.intersectionWith(colTestLine4));



      assertEquals(colTestp2, colTestLine1.intersectionWith(colTestLine5));
      assertEquals(colTestp2, colTestLine2.intersectionWith(colTestLine6));



      LineSegment2d line1 = new LineSegment2d(-10.0, 0.0, 10.0, 0.0);
      LineSegment2d line2 = new LineSegment2d(-10.0, 10.0, 10.0, 0.0);
      LineSegment2d line3 = new LineSegment2d(0.0, 10.0, 0.0, -10.0);
      LineSegment2d line4 = new LineSegment2d(0.0, -10.0, 0.0, 10.0);
      LineSegment2d line5 = new LineSegment2d(-10.0, 0.0, 10.0, 0.0);
      LineSegment2d line6 = new LineSegment2d(10.0, 0.0, -10.0, 0.0);
      LineSegment2d line7 = new LineSegment2d(10.0, 0.0, 20.0, 0.0);
      LineSegment2d line8 = new LineSegment2d(10.0, 0.0, -20.0, 0.0);
      LineSegment2d line9 = new LineSegment2d(10.1, 0.0, 20.0, 0.0);



      assertEquals(null, line5.intersectionWith(line1));
      assertEquals(null, line1.intersectionWith(line5));

      assertEquals(null, line1.intersectionWith(line6));


      assertEquals(new Point2d(10.0, 0.0), line1.intersectionWith(line2));
      assertEquals(new Point2d(0.0, 0.0), line1.intersectionWith(line3));
      assertEquals(new Point2d(0.0, 0.0), line1.intersectionWith(line4));

      assertEquals(new Point2d(10.0, 0.0), line1.intersectionWith(line7));
      assertEquals(null, line1.intersectionWith(line8));
      assertEquals(null, line1.intersectionWith(line9));
   }


}
