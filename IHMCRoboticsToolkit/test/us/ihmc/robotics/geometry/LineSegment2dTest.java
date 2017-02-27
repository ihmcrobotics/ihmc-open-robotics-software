package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.math.Epsilons;
import us.ihmc.robotics.random.RandomTools;

public class LineSegment2dTest
{
   private static Random ran = new Random(100L);
   private static LineSegment2d testSegment1;
   private static Point2D segment1Point1;
   private static Point2D segment1Point2;

   private static LineSegment2d testSegment2;
   private static Point2D segment2Point1;
   private static Point2D segment2Point2;

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

      segment1Point1 = new Point2D(x1, y1);
      segment1Point2 = new Point2D(x2, y2);
      testSegment1 = new LineSegment2d(segment1Point1, segment1Point2);


      do
      {
         x1 = ran.nextDouble() * 500 - 250;
         y1 = ran.nextDouble() * 500 - 250;
         x2 = ran.nextDouble() * 500 - 250;
         y2 = ran.nextDouble() * 500 - 250;
      }
      while ((x1 == x2) || (y1 == y2));

      segment2Point1 = new Point2D(x1, y1);
      segment2Point2 = new Point2D(x2, y2);
      testSegment2 = new LineSegment2d(segment2Point1, segment2Point2);
   }

   @After
   public void tearDown() throws Exception
   {
      testSegment1 = null;
      testSegment2 = null;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testDistancePoint2dLineSegment2d()
   {
      LineSegment2d line1 = new LineSegment2d(-10.0, 0.0, 10.0, 0.0);
      Point2D point1 = new Point2D(-10.0, 10.0);
      Point2D point2 = new Point2D(0.0, 10.0);
      Point2D point3 = new Point2D(0.0, -10.0);
      Point2D point4 = new Point2D(-10.0, 0.0);
      Point2D point5 = new Point2D(10.0, 0.0);
      Point2D point6 = new Point2D(10.5, 0.0);
      Point2D point7 = new Point2D(0.0, 1.2);
      Point2D point8 = new Point2D(10.1, 0.0);
      Point2D point9 = new Point2D(0.0, 0.0);
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
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
            assertEquals(test.getFirstEndpointCopy().getX(), x1, 0.001);
            assertEquals(test.getFirstEndpointCopy().getY(), y1, 0.001);
            assertEquals(test.getSecondEndpointCopy().getX(), x2, 0.001);
            assertEquals(test.getSecondEndpointCopy().getY(), y2, 0.001);
         }
      }

      // SHOULD FAIL

      try
      {
         double x1 = ran.nextDouble() * 500 - 250;
         double y1 = ran.nextDouble() * 500 - 250;
         new LineSegment2d(x1, y1, x1, y1);
      }
      catch (RuntimeException e)
      {
         fail = true;
      }

      assertTrue(fail);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
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
            Point2D[] points = new Point2D[2];
            points[0] = new Point2D(x1, y1);
            points[1] = new Point2D(x2, y2);
            LineSegment2d test = new LineSegment2d(points);
            assertEquals(test.getFirstEndpointCopy(), points[0]);
            assertEquals(test.getSecondEndpointCopy(), points[1]);
         }
      }

      // SHOULD FAIL

      try
      {
         double x1 = ran.nextDouble() * 500 - 250;
         double y1 = ran.nextDouble() * 500 - 250;
         Point2D[] points = new Point2D[2];
         points[0] = new Point2D(x1, y1);
         points[1] = new Point2D(x1, y1);
         new LineSegment2d(points);
      }
      catch (RuntimeException e)
      {
         fail = true;
      }

      assertTrue(fail);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
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
            Point2D[] points = new Point2D[2];
            points[0] = new Point2D(x1, y1);
            points[1] = new Point2D(x2, y2);
            LineSegment2d test = new LineSegment2d(points[0], points[1]);
            assertEquals(test.getFirstEndpointCopy(), points[0]);
            assertEquals(test.getSecondEndpointCopy(), points[1]);
         }
      }

      // SHOULD FAIL

      try
      {
         double x1 = ran.nextDouble() * 500 - 250;
         double y1 = ran.nextDouble() * 500 - 250;
         Point2D[] points = new Point2D[2];
         points[0] = new Point2D(x1, y1);
         points[1] = new Point2D(x1, y1);
         new LineSegment2d(points[0], points[1]);
      }
      catch (RuntimeException e)
      {
         fail = true;
      }

      assertTrue(fail);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
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
            Point2D[] points = new Point2D[2];
            points[0] = new Point2D(x1, y1);
            points[1] = new Point2D(x2, y2);
            LineSegment2d test = new LineSegment2d(points[0], points[1]);
            LineSegment2d test2 = new LineSegment2d(test);
            assertEquals(test.getFirstEndpointCopy(), test2.getFirstEndpointCopy());
            assertEquals(test.getSecondEndpointCopy(), test2.getSecondEndpointCopy());
         }
      }

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetEndpointsCopy()
   {
      Point2D[] pointsCopy = testSegment1.getEndpointsCopy();
      assertEquals(pointsCopy[0], testSegment1.getFirstEndpointCopy());
      assertEquals(pointsCopy[1], testSegment1.getSecondEndpointCopy());
      assertEquals(pointsCopy[0], segment1Point1);
      assertEquals(pointsCopy[1], segment1Point2);

      // make sure that chaning the copy does not change the origional
      pointsCopy[0].set(pointsCopy[0].getX() - 10.0, pointsCopy[0].getY() - 10.0);
      pointsCopy[1].set(pointsCopy[1].getX() - 10.0, pointsCopy[1].getY() - 10.0);

      assertFalse(pointsCopy[0].equals(testSegment1.getFirstEndpointCopy()));
      assertFalse(pointsCopy[1].equals(testSegment1.getSecondEndpointCopy()));
      assertFalse(pointsCopy[0].equals(segment1Point1));
      assertFalse(pointsCopy[1].equals(segment1Point2));

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetEndpointsPoint2dPoint2d()
   {
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetEndpoints()
   {
	   Point2DReadOnly[] points = testSegment1.getEndpoints();
      Point2D[] pointsCopy = testSegment1.getEndpointsCopy();
      assertEquals(pointsCopy[0], testSegment1.getFirstEndpointCopy());
      assertEquals(pointsCopy[1], testSegment1.getSecondEndpointCopy());
      assertEquals(pointsCopy[0], segment1Point1);
      assertEquals(pointsCopy[1], segment1Point2);

      // make sure that chaning the copy does not change the origional
      pointsCopy[0].set(pointsCopy[0].getX() - 10.0, pointsCopy[0].getY() - 10.0);
      pointsCopy[1].set(pointsCopy[1].getX() - 10.0, pointsCopy[1].getY() - 10.0);

      assertEquals(segment1Point1, points[0]);
      assertEquals(segment1Point2, points[1]);

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetFirstEndPointCopy()
   {
      Point2D pointCopy = testSegment1.getFirstEndpointCopy();
      assertEquals(pointCopy, segment1Point1);


      // make sure that chaning the copy does not change the origional
      pointCopy.set(pointCopy.getX() - 10.0, pointCopy.getY() - 10.0);


      assertFalse(pointCopy.equals(segment1Point1));


   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetSecondEndPointCopy()
   {
      Point2D pointCopy = testSegment1.getSecondEndpointCopy();
      assertEquals(pointCopy, segment1Point2);


      // make sure that chaning the copy does not change the origional
      pointCopy.set(pointCopy.getX() - 10.0, pointCopy.getY() - 10.0);


      assertFalse(pointCopy.equals(segment1Point2));

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetPoint2dPoint2d()
   {
      Point2D[] pointsCopy = testSegment1.getEndpointsCopy();

      pointsCopy[0].set(pointsCopy[0].getX() - 10.0, pointsCopy[0].getY() - 10.0);
      pointsCopy[1].set(pointsCopy[1].getX() - 10.0, pointsCopy[1].getY() - 10.0);

      testSegment1.set(pointsCopy[0], pointsCopy[1]);

      assertEquals(pointsCopy[0], testSegment1.getFirstEndpointCopy());
      assertEquals(pointsCopy[1], testSegment1.getSecondEndpointCopy());

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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetDoubleDoubleDoubleDouble()
   {
      Point2D[] pointsCopy = testSegment1.getEndpointsCopy();

      pointsCopy[0].set(pointsCopy[0].getX() - 10.0, pointsCopy[0].getY() - 10.0);
      pointsCopy[1].set(pointsCopy[1].getX() - 10.0, pointsCopy[1].getY() - 10.0);

      testSegment1.set(pointsCopy[0].getX(), pointsCopy[0].getY(), pointsCopy[1].getX(), pointsCopy[1].getY());

      assertEquals(pointsCopy[0], testSegment1.getFirstEndpointCopy());
      assertEquals(pointsCopy[1], testSegment1.getSecondEndpointCopy());
      boolean throwEx = false;
      try
      {
         pointsCopy[0] = pointsCopy[1];
         testSegment1.set(pointsCopy[0].getX(), pointsCopy[0].getY(), pointsCopy[1].getX(), pointsCopy[1].getY());
      }
      catch (Exception e)
      {
         throwEx = true;
      }

      assertTrue(throwEx);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetPoint2dArray()
   {
      Point2D[] pointsCopy = testSegment1.getEndpointsCopy();



      pointsCopy[0].set(pointsCopy[0].getX() - 10.0, pointsCopy[0].getY() - 10.0);
      pointsCopy[1].set(pointsCopy[1].getX() - 10.0, pointsCopy[1].getY() - 10.0);

      testSegment1.set(pointsCopy);

      assertEquals(pointsCopy[0], testSegment1.getFirstEndpointCopy());
      assertEquals(pointsCopy[1], testSegment1.getSecondEndpointCopy());
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSetLineSegment2d()
   {
      testSegment1.set(testSegment2);

      assertEquals(testSegment2.getFirstEndpointCopy(), testSegment1.getFirstEndpointCopy());
      assertEquals(testSegment2.getSecondEndpointCopy(), testSegment1.getSecondEndpointCopy());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testFlipDirection()
   {
      testSegment1.flipDirection();
      assertEquals(segment1Point2, testSegment1.getFirstEndpointCopy());
      assertEquals(segment1Point1, testSegment1.getSecondEndpointCopy());

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testMidpoint()
   {
      Point2D midPoint = testSegment1.midpoint();
      assertEquals(midPoint.distance(testSegment1.getFirstEndpointCopy()), midPoint.distance(testSegment1.getSecondEndpointCopy()), 0.001);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPercentageAlongLineSegment()
   {
      LineSegment2d line1 = new LineSegment2d(-10, 0, 10, 0);
      assertEquals(0.5, line1.percentageAlongLineSegment(new Point2D(0.0, 0.0)), 0.001);
      assertEquals(0.0, line1.percentageAlongLineSegment(new Point2D(-10.0, 0.0)), 0.001);
      assertEquals(1.0, line1.percentageAlongLineSegment(new Point2D(10.0, 0.0)), 0.001);
      assertEquals(0.5, line1.percentageAlongLineSegment(new Point2D(0.0, 5.0)), 0.001);

      Random random = new Random(23424L);

      // Test on line segment
      for (int i = 0; i < 1000; i++)
      {
         Point2D lineSegmentStart = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Point2D lineSegmentEnd = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         LineSegment2d lineSegment = new LineSegment2d(lineSegmentStart, lineSegmentEnd);

         Point2D pointOnLineSegment = new Point2D();

         // Test between end points
         double expectedPercentage = RandomNumbers.nextDouble(random, 0.0, 1.0);
         pointOnLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         double actualPercentage = lineSegment.percentageAlongLineSegment(pointOnLineSegment);
         assertEquals(expectedPercentage, actualPercentage, Epsilons.ONE_TRILLIONTH);

         // Test before end points
         expectedPercentage = RandomNumbers.nextDouble(random, -10.0, 0.0);
         pointOnLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         actualPercentage = lineSegment.percentageAlongLineSegment(pointOnLineSegment);
         assertEquals(expectedPercentage, actualPercentage, Epsilons.ONE_TRILLIONTH);

         // Test after end points
         expectedPercentage = RandomNumbers.nextDouble(random, 1.0, 10.0);
         pointOnLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         actualPercentage = lineSegment.percentageAlongLineSegment(pointOnLineSegment);
         assertEquals(expectedPercentage, actualPercentage, Epsilons.ONE_TRILLIONTH);
      }

      // Test off line segment
      for (int i = 0; i < 1000; i++)
      {
         Point2D lineSegmentStart = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Point2D lineSegmentEnd = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         LineSegment2d lineSegment = new LineSegment2d(lineSegmentStart, lineSegmentEnd);

         Point2D pointOffLineSegment = new Point2D();
         Vector2D lineSegmentDirection = new Vector2D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         Vector2D orthogonal = GeometryTools.getPerpendicularVector(lineSegmentDirection);
         orthogonal.normalize();

         // Test between end points
         double expectedPercentage = RandomNumbers.nextDouble(random, 0.0, 1.0);
         pointOffLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         pointOffLineSegment.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, pointOffLineSegment);
         double actualPercentage = lineSegment.percentageAlongLineSegment(pointOffLineSegment);
         assertEquals(expectedPercentage, actualPercentage, Epsilons.ONE_TRILLIONTH);

         // Test before end points
         expectedPercentage = RandomNumbers.nextDouble(random, -10.0, 0.0);
         pointOffLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         pointOffLineSegment.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, pointOffLineSegment);
         actualPercentage = lineSegment.percentageAlongLineSegment(pointOffLineSegment);
         assertEquals(expectedPercentage, actualPercentage, Epsilons.ONE_TRILLIONTH);

         // Test after end points
         expectedPercentage = RandomNumbers.nextDouble(random, 1.0, 10.0);
         pointOffLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         pointOffLineSegment.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, pointOffLineSegment);
         actualPercentage = lineSegment.percentageAlongLineSegment(pointOffLineSegment);
         assertEquals(expectedPercentage, actualPercentage, Epsilons.ONE_TRILLIONTH);
      }
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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIntersectionWithLineSegment2d1()
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



      assertEquals(new Point2D(-10.0, 0.0), line1.intersectionWith(line1));
      assertEquals(new Point2D(-10.0, 0.0), line1.intersectionWith(line5));
      assertEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line6));
      assertEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line2));
      assertEquals(new Point2D(0.0, 0.0), line1.intersectionWith(line3));
      assertEquals(new Point2D(0.0, 0.0), line1.intersectionWith(line4));


      assertEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line7));
      assertEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line8));
      assertEquals(null, line1.intersectionWith(line9));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIntersectionWithLineSegment2d2()
   {
      Random random = new Random(3242L);

      for (int i = 0; i < 100; i++)
      {
         Point2D lineSegmentStart1 = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Point2D lineSegmentEnd1 = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.interpolate(lineSegmentStart1, lineSegmentEnd1, RandomNumbers.nextDouble(random, 0.0, 1.0));

         Vector2D lineDirection2 = RandomTools.generateRandomVector2d(random, 1.0);

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         // Expecting intersection
         lineSegmentStart2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         lineSegmentEnd2.scaleAdd(RandomNumbers.nextDouble(random, -10.0, 0.0), lineDirection2, expectedIntersection);
         assertAllCombinationsOfIntersectionWith(expectedIntersection, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);

         // Not expecting intersection
         lineSegmentStart2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         lineSegmentEnd2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         assertOnlyExistenceOfIntersectionAllCombinations(false, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      }

      // Test intersection at one of the end points
      for (int i = 0; i < 100; i++)
      {
         Point2D lineSegmentStart1 = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Point2D lineSegmentEnd1 = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);

         Point2D expectedIntersection = new Point2D(lineSegmentStart1);

         Vector2D lineDirection2 = RandomTools.generateRandomVector2d(random, 1.0);

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         // Not expecting intersection
         lineSegmentStart2.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         lineSegmentEnd2.scaleAdd(RandomNumbers.nextDouble(random, -10.0, 0.0), lineDirection2, expectedIntersection);
         assertAllCombinationsOfIntersectionWith(expectedIntersection, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      }

      // Test with parallel/collinear line segments
      for (int i = 0; i < 100; i++)
      {
         Point2D lineSegmentStart1 = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Point2D lineSegmentEnd1 = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         double alpha1 = RandomNumbers.nextDouble(random, 2.0);
         double alpha2 = RandomNumbers.nextDouble(random, 2.0);

         // Make the second line segment collinear to the first one
         lineSegmentStart2.interpolate(lineSegmentStart1, lineSegmentEnd1, alpha1);
         lineSegmentEnd2.interpolate(lineSegmentStart1, lineSegmentEnd1, alpha2);

         if ((0.0 < alpha1 && alpha1 < 1.0) || (0.0 < alpha2 && alpha2 < 1.0) || alpha1 * alpha2 < 0.0)
         {
            assertOnlyExistenceOfIntersectionAllCombinations(true, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
         }
         else
         {
            assertOnlyExistenceOfIntersectionAllCombinations(false, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
         }

         // Shift the second line segment such that it becomes only parallel to the first.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(lineSegmentEnd1, lineSegmentStart1);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = RandomNumbers.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart2.scaleAdd(distance, orthogonal, lineSegmentStart2);
         lineSegmentEnd2.scaleAdd(distance, orthogonal, lineSegmentEnd2);
         assertOnlyExistenceOfIntersectionAllCombinations(false, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      }
   }

   private void assertOnlyExistenceOfIntersectionAllCombinations(boolean intersectionExist, Point2D lineSegmentStart1, Point2D lineSegmentEnd1, Point2D lineSegmentStart2, Point2D lineSegmentEnd2)
   {
      LineSegment2d lineSegment1 = new LineSegment2d(lineSegmentStart1, lineSegmentEnd1);
      LineSegment2d reverseLineSegment1 = new LineSegment2d(lineSegmentEnd1, lineSegmentStart1);
      LineSegment2d lineSegment2 = new LineSegment2d(lineSegmentStart2, lineSegmentEnd2);
      LineSegment2d reverseLineSegment2 = new LineSegment2d(lineSegmentEnd2, lineSegmentStart2);

      if (intersectionExist)
      {
         assertNotNull(lineSegment1.intersectionWith(lineSegment2));
         assertNotNull(lineSegment1.intersectionWith(reverseLineSegment2));
         assertNotNull(reverseLineSegment1.intersectionWith(lineSegment2));
         assertNotNull(reverseLineSegment1.intersectionWith(reverseLineSegment2));

         assertNotNull(lineSegment2.intersectionWith(lineSegment1));
         assertNotNull(lineSegment2.intersectionWith(reverseLineSegment1));
         assertNotNull(reverseLineSegment2.intersectionWith(lineSegment1));
         assertNotNull(reverseLineSegment2.intersectionWith(reverseLineSegment1));
      }
      else
      {
         assertNull(lineSegment1.intersectionWith(lineSegment2));
         assertNull(lineSegment1.intersectionWith(reverseLineSegment2));
         assertNull(reverseLineSegment1.intersectionWith(lineSegment2));
         assertNull(reverseLineSegment1.intersectionWith(reverseLineSegment2));

         assertNull(lineSegment2.intersectionWith(lineSegment1));
         assertNull(lineSegment2.intersectionWith(reverseLineSegment1));
         assertNull(reverseLineSegment2.intersectionWith(lineSegment1));
         assertNull(reverseLineSegment2.intersectionWith(reverseLineSegment1));
      }
   }

   private void assertAllCombinationsOfIntersectionWith(Point2D expectedIntersection, Point2D lineSegmentStart1, Point2D lineSegmentEnd1, Point2D lineSegmentStart2, Point2D lineSegmentEnd2)
   {
      double epsilon = Epsilons.ONE_TRILLIONTH;

      LineSegment2d lineSegment1 = new LineSegment2d(lineSegmentStart1, lineSegmentEnd1);
      LineSegment2d reverseLineSegment1 = new LineSegment2d(lineSegmentEnd1, lineSegmentStart1);
      LineSegment2d lineSegment2 = new LineSegment2d(lineSegmentStart2, lineSegmentEnd2);
      LineSegment2d reverseLineSegment2 = new LineSegment2d(lineSegmentEnd2, lineSegmentStart2);

      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, lineSegment1.intersectionWith(lineSegment2), epsilon);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, lineSegment1.intersectionWith(reverseLineSegment2), epsilon);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, reverseLineSegment1.intersectionWith(lineSegment2), epsilon);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, reverseLineSegment1.intersectionWith(reverseLineSegment2), epsilon);

      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, lineSegment2.intersectionWith(lineSegment1), epsilon);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, lineSegment2.intersectionWith(reverseLineSegment1), epsilon);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, reverseLineSegment2.intersectionWith(lineSegment1), epsilon);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, reverseLineSegment2.intersectionWith(reverseLineSegment1), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIntersectionWithLine2d2()
   {
      double epsilon = Epsilons.ONE_TRILLIONTH;
      Random random = new Random(23423L);

      for (int i = 0; i < 100; i++)
      {
         Point2D lineSegmentStart = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Point2D lineSegmentEnd = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         LineSegment2d lineSegment = new LineSegment2d(lineSegmentStart, lineSegmentEnd);
         LineSegment2d reverseLineSegment = new LineSegment2d(lineSegmentEnd, lineSegmentStart);

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.interpolate(lineSegmentStart, lineSegmentEnd, RandomNumbers.nextDouble(random, 0.0, 1.0));

         Point2D pointOnLine = new Point2D(expectedIntersection);
         Vector2D lineDirection = RandomTools.generateRandomVector2d(random, 1.0);

         // Expecting intersection
         Point2D actualIntersection = lineSegment.intersectionWith(new Line2d(pointOnLine, lineDirection));
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         actualIntersection = reverseLineSegment.intersectionWith(new Line2d(pointOnLine, lineDirection));
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         pointOnLine.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         actualIntersection = lineSegment.intersectionWith(new Line2d(pointOnLine, lineDirection));
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         actualIntersection = reverseLineSegment.intersectionWith(new Line2d(pointOnLine, lineDirection));
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      }

      // Make the intersection happen outside the line segment
      for (int i = 0; i < 100; i++)
      {
         Point2D lineSegmentStart = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Point2D lineSegmentEnd = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         LineSegment2d lineSegment = new LineSegment2d(lineSegmentStart, lineSegmentEnd);
         LineSegment2d reverseLineSegment = new LineSegment2d(lineSegmentEnd, lineSegmentStart);
         
         Point2D pointOnLine = new Point2D();
         Vector2D lineDirection = RandomTools.generateRandomVector2d(random, 1.0);

         Point2D lineLineIntersection = new Point2D();
         lineLineIntersection.interpolate(lineSegmentStart, lineSegmentEnd, RandomNumbers.nextDouble(random, 1.0, 2.0));
         pointOnLine.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection, lineLineIntersection);
         Point2D actualIntersection = lineSegment.intersectionWith(new Line2d(pointOnLine, lineDirection));
         assertNull(actualIntersection);

         actualIntersection = reverseLineSegment.intersectionWith(new Line2d(pointOnLine, lineDirection));
         assertNull(actualIntersection);

         lineLineIntersection.interpolate(lineSegmentStart, lineSegmentEnd, RandomNumbers.nextDouble(random, -1.0, 0.0));
         pointOnLine.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection, lineLineIntersection);
         actualIntersection = lineSegment.intersectionWith(new Line2d(pointOnLine, lineDirection));
         assertNull(actualIntersection);

         actualIntersection = reverseLineSegment.intersectionWith(new Line2d(pointOnLine, lineDirection));
         assertNull(actualIntersection);
      }

      // Make the intersection happen on each end point of the line segment
      for (int i = 0; i < 100; i++)
      {
         Point2D lineSegmentStart = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Point2D lineSegmentEnd = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         LineSegment2d lineSegment = new LineSegment2d(lineSegmentStart, lineSegmentEnd);
         LineSegment2d reverseLineSegment = new LineSegment2d(lineSegmentEnd, lineSegmentStart);

         Point2D pointOnLine = new Point2D();
         Vector2D lineDirection = RandomTools.generateRandomVector2d(random, 1.0);

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.set(lineSegmentStart);
         pointOnLine.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         Point2D actualIntersection = lineSegment.intersectionWith(new Line2d(pointOnLine, lineDirection));
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         actualIntersection = reverseLineSegment.intersectionWith(new Line2d(pointOnLine, lineDirection));
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         expectedIntersection.set(lineSegmentEnd);
         pointOnLine.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         actualIntersection = lineSegment.intersectionWith(new Line2d(pointOnLine, lineDirection));
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         actualIntersection = reverseLineSegment.intersectionWith(new Line2d(pointOnLine, lineDirection));
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      }

      // Make the line segment and the line parallel not collinear.
      for (int i = 0; i < 100; i++)
      {
         Point2D lineSegmentStart = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Point2D lineSegmentEnd = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         LineSegment2d lineSegment = new LineSegment2d(lineSegmentStart, lineSegmentEnd);
         
         Point2D pointOnLine = new Point2D(lineSegmentStart);
         Vector2D lineDirection = new Vector2D();
         lineDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineDirection.normalize();
         if (random.nextBoolean())
            lineDirection.negate();

         Vector2D orthogonal = new Vector2D(-lineDirection.getY(), lineDirection.getY());

         pointOnLine.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), orthogonal, pointOnLine);
         pointOnLine.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection, pointOnLine);
         Point2D actualIntersection = lineSegment.intersectionWith(new Line2d(pointOnLine, lineDirection));
         assertNull(actualIntersection);
      }

      // Make the line segment and the line collinear.
      for (int i = 0; i < 100; i++)
      {
         Point2D lineSegmentStart = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Point2D lineSegmentEnd = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         LineSegment2d lineSegment = new LineSegment2d(lineSegmentStart, lineSegmentEnd);
         LineSegment2d reverseLineSegment = new LineSegment2d(lineSegmentEnd, lineSegmentStart);
         
         Point2D pointOnLine = new Point2D(lineSegmentStart);
         Vector2D lineDirection = new Vector2D();
         lineDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineDirection.normalize();
         if (random.nextBoolean())
            lineDirection.negate();

         pointOnLine.scaleAdd(RandomNumbers.nextDouble(random, 0.0, 10.0), lineDirection, pointOnLine);
         Point2D actualIntersection = lineSegment.intersectionWith(new Line2d(pointOnLine, lineDirection));
         EuclidCoreTestTools.assertTuple2DEquals(lineSegmentStart, actualIntersection, epsilon);
         actualIntersection = reverseLineSegment.intersectionWith(new Line2d(pointOnLine, lineDirection));
         EuclidCoreTestTools.assertTuple2DEquals(lineSegmentEnd, actualIntersection, epsilon);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testDistancePoint2d()
   {
      LineSegment2d line1 = new LineSegment2d(-10.0, 0.0, 10.0, 0.0);
      Point2D point1 = new Point2D(-10.0, 10.0);
      Point2D point2 = new Point2D(0.0, 10.0);
      Point2D point3 = new Point2D(0.0, -10.0);
      Point2D point4 = new Point2D(-10.0, 0.0);
      Point2D point5 = new Point2D(10.0, 0.0);
      Point2D point6 = new Point2D(10.5, 0.0);
      Point2D point7 = new Point2D(0.0, 1.2);
      Point2D point8 = new Point2D(10.1, 0.0);
      Point2D point9 = new Point2D(0.0, 0.0);
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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testShiftToLeftAndRightCopy()
   {
      double distanceToShift = 0.2;
      double epsilon = 1e-7;

      // Pointing straight up:
      LineSegment2d lineSegment = new LineSegment2d(0.0, 0.0, 0.0, 1.0);
      LineSegment2d shiftedLineSegment = lineSegment.shiftToRightCopy(distanceToShift);

      Point2D firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      Point2D secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(distanceToShift, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(0.0, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(distanceToShift, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(1.0, secondShiftedEndpoint.getY(), epsilon);

      shiftedLineSegment = lineSegment.shiftToLeftCopy(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(-distanceToShift, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(0.0, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(-distanceToShift, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(1.0, secondShiftedEndpoint.getY(), epsilon);

      // Pointing straight along x:
      lineSegment = new LineSegment2d(0.0, 0.0, 1.0, 0.0);
      shiftedLineSegment = lineSegment.shiftToRightCopy(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(0.0, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(-distanceToShift, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(1.0, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(-distanceToShift, secondShiftedEndpoint.getY(), epsilon);

      shiftedLineSegment = lineSegment.shiftToLeftCopy(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(0.0, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(distanceToShift, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(1.0, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(distanceToShift, secondShiftedEndpoint.getY(), epsilon);

      // Pointing at (1,1)
      lineSegment = new LineSegment2d(0.0, 0.0, 1.0, 1.0);
      shiftedLineSegment = lineSegment.shiftToRightCopy(distanceToShift);
      double distanceAtFortyFiveDegrees = distanceToShift * Math.sqrt(2.0) / 2.0;

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(distanceAtFortyFiveDegrees, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(-distanceAtFortyFiveDegrees, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(1.0 + distanceAtFortyFiveDegrees, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(1.0 - distanceAtFortyFiveDegrees, secondShiftedEndpoint.getY(), epsilon);

      shiftedLineSegment = lineSegment.shiftToLeftCopy(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(-distanceAtFortyFiveDegrees, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(distanceAtFortyFiveDegrees, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(1.0 - distanceAtFortyFiveDegrees, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(1.0 + distanceAtFortyFiveDegrees, secondShiftedEndpoint.getY(), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testShiftToLeftAndRight()
   {
      double distanceToShift = 0.2;
      double epsilon = 1e-7;

      // Pointing straight up:
      LineSegment2d lineSegment = new LineSegment2d(0.0, 0.0, 0.0, 1.0);
      LineSegment2d shiftedLineSegment = new LineSegment2d(lineSegment);
      shiftedLineSegment.shiftToRight(distanceToShift);

      Point2D firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      Point2D secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(distanceToShift, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(0.0, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(distanceToShift, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(1.0, secondShiftedEndpoint.getY(), epsilon);

      shiftedLineSegment.set(lineSegment);
      shiftedLineSegment.shiftToLeft(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(-distanceToShift, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(0.0, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(-distanceToShift, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(1.0, secondShiftedEndpoint.getY(), epsilon);

      // Pointing straight along x:
      lineSegment = new LineSegment2d(0.0, 0.0, 1.0, 0.0);
      shiftedLineSegment.set(lineSegment);
      shiftedLineSegment.shiftToRight(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(0.0, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(-distanceToShift, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(1.0, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(-distanceToShift, secondShiftedEndpoint.getY(), epsilon);

      shiftedLineSegment.set(lineSegment);
      shiftedLineSegment.shiftToLeft(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(0.0, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(distanceToShift, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(1.0, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(distanceToShift, secondShiftedEndpoint.getY(), epsilon);

      // Pointing at (1,1)
      lineSegment = new LineSegment2d(0.0, 0.0, 1.0, 1.0);
      shiftedLineSegment.set(lineSegment);
      shiftedLineSegment.shiftToRight(distanceToShift);

      double distanceAtFortyFiveDegrees = distanceToShift * Math.sqrt(2.0) / 2.0;

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(distanceAtFortyFiveDegrees, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(-distanceAtFortyFiveDegrees, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(1.0 + distanceAtFortyFiveDegrees, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(1.0 - distanceAtFortyFiveDegrees, secondShiftedEndpoint.getY(), epsilon);

      shiftedLineSegment.set(lineSegment);
      shiftedLineSegment.shiftToLeft(distanceToShift);

      firstShiftedEndpoint = shiftedLineSegment.getFirstEndpointCopy();
      secondShiftedEndpoint = shiftedLineSegment.getSecondEndpointCopy();

      assertEquals(-distanceAtFortyFiveDegrees, firstShiftedEndpoint.getX(), epsilon);
      assertEquals(distanceAtFortyFiveDegrees, firstShiftedEndpoint.getY(), epsilon);
      assertEquals(1.0 - distanceAtFortyFiveDegrees, secondShiftedEndpoint.getX(), epsilon);
      assertEquals(1.0 + distanceAtFortyFiveDegrees, secondShiftedEndpoint.getY(), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIsPointOnLeftRightSide()
   {
      LineSegment2d lineSegment = new LineSegment2d(0.0, 0.0, 0.0, 1.0);

      Point2D point = new Point2D(0.1, 0.5);
      assertFalse(lineSegment.isPointOnLeftSideOfLineSegment(point));
      assertTrue(lineSegment.isPointOnRightSideOfLineSegment(point));

      point = new Point2D(-0.1, 0.5);
      assertTrue(lineSegment.isPointOnLeftSideOfLineSegment(point));
      assertFalse(lineSegment.isPointOnRightSideOfLineSegment(point));

      lineSegment = new LineSegment2d(0.0, 2.0, 4.0, 6.0);

      point = new Point2D(0.0, 0.0);
      assertTrue(lineSegment.isPointOnRightSideOfLineSegment(point));

      point = new Point2D(4.1, 6.0);
      assertTrue(lineSegment.isPointOnRightSideOfLineSegment(point));

      point = new Point2D(3.9, 6.0);
      assertTrue(lineSegment.isPointOnLeftSideOfLineSegment(point));

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testOrthogonalProjectionCopyPoint2dLineSegment2d()
   {
      Point2D startPoint = new Point2D(-10.0, 0.0);
      Point2D endPoint = new Point2D(10.0, 0.0);
      LineSegment2d line1 = new LineSegment2d(startPoint, endPoint);

      Point2D origionalPoint = new Point2D(-20.0, 10.0);
      Point2D projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(startPoint, projectedPoint);

      origionalPoint = new Point2D(-20.0, -10.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(startPoint, projectedPoint);

      origionalPoint = new Point2D(-20.0, 0.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(startPoint, projectedPoint);

      origionalPoint = new Point2D(20.0, 10.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(endPoint, projectedPoint);

      origionalPoint = new Point2D(20.0, -10.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(endPoint, projectedPoint);

      origionalPoint = new Point2D(20.0, 0.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(endPoint, projectedPoint);

      origionalPoint = new Point2D(0.0, 10.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(new Point2D(0.0, 0.0), projectedPoint);

      origionalPoint = new Point2D(0.0, -10.0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(new Point2D(0.0, 0.0), projectedPoint);


      origionalPoint = new Point2D(5.0, 0);
      projectedPoint = line1.orthogonalProjectionCopy(origionalPoint);
      assertEquals(new Point2D(5.0, 0.0), projectedPoint);

      Random random = new Random(2342L);

      for (int i = 0; i < 1000; i++)
      {
         Point2D lineSegmentStart = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         Point2D lineSegmentEnd = RandomTools.generateRandomPoint2d(random, 10.0, 10.0);
         LineSegment2d lineSegment = new LineSegment2d(lineSegmentStart, lineSegmentEnd);
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(lineSegmentEnd, lineSegmentStart);
         GeometryTools.getPerpendicularVector(orthogonal, orthogonal);
         orthogonal.normalize();
         Point2D expectedProjection = new Point2D();
         Point2D testPoint = new Point2D();

         // Between end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, RandomNumbers.nextDouble(random, 0.0, 1.0));
         testPoint.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, expectedProjection);
         Point2D actualProjection = lineSegment.orthogonalProjectionCopy(testPoint);
         EuclidCoreTestTools.assertTuple2DEquals(expectedProjection, actualProjection, Epsilons.ONE_TRILLIONTH);

         // Before end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, RandomNumbers.nextDouble(random, -10.0, 0.0));
         testPoint.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, expectedProjection);
         expectedProjection.set(lineSegmentStart);
         actualProjection = lineSegment.orthogonalProjectionCopy(testPoint);
         EuclidCoreTestTools.assertTuple2DEquals(expectedProjection, actualProjection, Epsilons.ONE_TRILLIONTH);

         // After end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, RandomNumbers.nextDouble(random, 1.0, 10.0));
         testPoint.scaleAdd(RandomNumbers.nextDouble(random, 10.0), orthogonal, expectedProjection);
         expectedProjection.set(lineSegmentEnd);
         actualProjection = lineSegment.orthogonalProjectionCopy(testPoint);
         EuclidCoreTestTools.assertTuple2DEquals(expectedProjection, actualProjection, Epsilons.ONE_TRILLIONTH);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIntersectionLine2dLineSegment2d()
   {
	   double epsilon = Epsilons.ONE_TRILLIONTH;

      LineSegment2d line1 = new LineSegment2d(-10.0, 0.0, 10.0, 0.0);
      Line2d line2 = new Line2d(new Point2D(-10.0, 10.0), new Point2D(10.0, 0.0));
      Line2d line3 = new Line2d(new Point2D(0.0, 10.0), new Point2D(0.0, -10.0));
      Line2d line4 = new Line2d(new Point2D(0.0, -10.0), new Point2D(0.0, 10.0));
      Line2d line5 = new Line2d(new Point2D(-10.0, 0.0), new Point2D(10.0, 0.0));
      Line2d line6 = new Line2d(new Point2D(10.0, 0.0), new Point2D(-10.0, 0.0));
      Line2d line7 = new Line2d(new Point2D(10.0, 0.0), new Point2D(20.0, 0.0));
      Line2d line8 = new Line2d(new Point2D(10.0, 0.0), new Point2D(-20.0, 0.0));
      Line2d line9 = new Line2d(new Point2D(10.1, 0.0), new Point2D(20.0, 0.0));
      Line2d line10 = new Line2d(new Point2D(10.0, 0.0), new Point2D(20.0, 1.0));


      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-10.0, 0.0), line1.intersectionWith(line5), epsilon);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-10.0, 0.0), line1.intersectionWith(line6), epsilon);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line2), epsilon);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line10), epsilon);

      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.0, 0.0), line1.intersectionWith(line3), epsilon);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(0.0, 0.0), line1.intersectionWith(line4), epsilon);


      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-10.0, 0.0), line1.intersectionWith(line7), epsilon);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-10.0, 0.0), line1.intersectionWith(line8), epsilon);
      EuclidCoreTestTools.assertTuple2DEquals(new Point2D(-10.0, 0.0), line1.intersectionWith(line9), epsilon);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIntersectionLineSegment2dLineSegment2d()
   {
      Point2D colTestp1 = new Point2D(1.5, -5);
      Point2D colTestp2 = new Point2D(1.5, 0);
      Point2D colTestp3 = new Point2D(1.5, 5);

      LineSegment2d colTestLine1 = new LineSegment2d(colTestp1, colTestp2);
      LineSegment2d colTestLine2 = new LineSegment2d(colTestp2, colTestp1);

      LineSegment2d colTestLine3 = new LineSegment2d(colTestp1, colTestp3);
      LineSegment2d colTestLine4 = new LineSegment2d(colTestp3, colTestp1);

      LineSegment2d colTestLine5 = new LineSegment2d(colTestp2, colTestp3);
      LineSegment2d colTestLine6 = new LineSegment2d(colTestp3, colTestp2);


      assertEquals(colTestp2, colTestLine1.intersectionWith(colTestLine2));
      assertEquals(colTestp1, colTestLine1.intersectionWith(colTestLine3));
      assertEquals(colTestp1, colTestLine1.intersectionWith(colTestLine4));
      assertEquals(colTestp1, colTestLine2.intersectionWith(colTestLine4));



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



      assertEquals(new Point2D(-10.0, 0.0), line5.intersectionWith(line1));
      assertEquals(new Point2D(-10.0, 0.0), line1.intersectionWith(line5));

      assertEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line6));


      assertEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line2));
      assertEquals(new Point2D(0.0, 0.0), line1.intersectionWith(line3));
      assertEquals(new Point2D(0.0, 0.0), line1.intersectionWith(line4));

      assertEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line7));
      assertEquals(new Point2D(10.0, 0.0), line1.intersectionWith(line8));
      assertEquals(null, line1.intersectionWith(line9));
   }


}
