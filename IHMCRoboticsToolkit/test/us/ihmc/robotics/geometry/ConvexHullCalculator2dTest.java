package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.vecmath.Point2d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */ 
public class ConvexHullCalculator2dTest
{
   private static final boolean VERBOSE = false;
   
   private ArrayList<Point2d> coincidalPoints, twoPointsEqualX, fourPoints, eightPoints, randomList;
   private Random random;
   private final int RANDOMLISTSIZEMAX = 100;

//   public TestConvexHullCalculator2d(String name)
//   {
//      setUp();
//   }

   @Before
   public void setUp()
   {
      random = new Random(100L);

      coincidalPoints = new ArrayList<Point2d>();
      coincidalPoints.add(new Point2d(1.0, 2.0));
      coincidalPoints.add(new Point2d(1.0, 2.0));

      twoPointsEqualX = new ArrayList<Point2d>();
      twoPointsEqualX.add(new Point2d(1.0, 2.0));
      twoPointsEqualX.add(new Point2d(1.0, 4.0));

      fourPoints = new ArrayList<Point2d>();
      fourPoints.add(new Point2d(0.0, 0.0));
      fourPoints.add(new Point2d(1.0, 0.0));
      fourPoints.add(new Point2d(0.5, 0.5));
      fourPoints.add(new Point2d(2.0, 1.0));

      eightPoints = new ArrayList<Point2d>();
      eightPoints.add(new Point2d(3.0, 3.0));
      eightPoints.add(new Point2d(0.0, 6.0));
      eightPoints.add(new Point2d(4.0, 6.0));
      eightPoints.add(new Point2d(4.0, 8.0));
      eightPoints.add(new Point2d(6.0, 1.0));
      eightPoints.add(new Point2d(-2.0, 3.0));
      eightPoints.add(new Point2d(2.0, -3.0));
      eightPoints.add(new Point2d(6.0, 0.0));

      randomList = new ArrayList<Point2d>();
   }

   @After
   public void tearDown()
   {

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIsConvexAndClockwiseBadCase()
   {
      ArrayList<Point2d> points = new ArrayList<Point2d>();
      points.add(new Point2d(-0.09138473539252934, 0.15067644107190148));
      points.add(new Point2d(-0.2213792355594174, 0.2372383866486496));
      points.add(new Point2d(0.49398049469327043, 0.15061199094999478));
      points.add(new Point2d(0.03518927981171038, -0.4438475980026291));

      boolean convexAndClockwise = ConvexHullCalculator2d.isConvexAndClockwise(points);
      ArrayList<Point2d> convexHull = new ArrayList<Point2d>();
      ConvexHullCalculator2d.getConvexHull(convexHull, points);
      if (convexAndClockwise)
         assertEquals(points.size(), convexHull.size());
      else
         assertTrue(points.size() > convexHull.size());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConvexAndClockwiseVersusConvexHull()
   {
      int minPoints = 3;
      int maxPoints = 12;
      int nTests = 1000;
      for (int testNumber = 0; testNumber < nTests; testNumber++)
      {
         ArrayList<Point2d> points = new ArrayList<Point2d>();
         int nPoints = minPoints + random.nextInt(maxPoints - minPoints);
         for (int i = 0; i < nPoints; i++)
         {
            points.add(new Point2d(random.nextDouble() - 0.5, random.nextDouble() - 0.5));
         }

         ConvexPolygon2d poly = new ConvexPolygon2d(points);
         List<Point2d> polyPoints = new ArrayList<Point2d>();
         for (int i = 0; i < poly.getNumberOfVertices(); i++)
            polyPoints.add(poly.getVertex(i));
         
         assertTrue(InPlaceConvexHullCalculator2d.isConvexAndClockwise(polyPoints));
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimplified7PointProblem()
   {
      ArrayList<Point2d> points = new ArrayList<Point2d>();

//    points.add(new Point2d(0.0, 1.0));
//    points.add(new Point2d(0.05079999999998946, 0.1428749999998843));
//    points.add(new Point2d(0.050799999999989486, -0.041274999999863206));
//    points.add(new Point2d(0.050800000000010524, 0.04127499999988427));
//    points.add(new Point2d(0.05080000000001055, -0.1428749999998632));

      points.add(new Point2d(0.0, 1.0));
      points.add(new Point2d(0.0508 + (-1.054017984003508E-14), 0.1428749999998843));
      points.add(new Point2d(0.0508 + (-1.0512424264419451E-14), -0.041274999999863206));
      points.add(new Point2d(0.0508 + (1.0526302052227265E-14), 0.04127499999988427));
      points.add(new Point2d(0.0508 + (1.0554057627842894E-14), -0.1428749999998632));

      if (VERBOSE)
      {
         for (int i = 0; i < points.size(); i++)
         {
            for (int j = 0; j < points.size(); j++)
            {
               System.out.println("Slope between " + i + " and " + j + ": " + slope(points.get(i), points.get(j)));
            }

            System.out.println();
         }
      }

      ConvexPolygon2d poly = new ConvexPolygon2d(points);
      List<Point2d> polyPoints = new ArrayList<Point2d>();
      for (int i = 0; i < poly.getNumberOfVertices(); i++)
         polyPoints.add(poly.getVertex(i));
      
      boolean convexAndClockwise = InPlaceConvexHullCalculator2d.isConvexAndClockwise(polyPoints);

      assertTrue("Result not convex and clockwise. convexHull = " + polyPoints, convexAndClockwise);
   }

   private double slope(Point2d point1, Point2d point2)
   {
      return (point2.getY() - point1.getY()) / (point2.getX() - point1.getX());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void test7PointProblem()
   {
      double[][] points = new double[][]
      {
         {-0.05080000000001055, 0.1428749999998632}, {0.12191999999998944, 0.14287499999989903}, {0.1219200000000105, 0.041274999999899024},
         {0.12192000000001053, -0.14287499999984846}, {-0.05079999999998946, -0.1428749999998843}, {-0.050799999999989486, 0.041274999999863206},
         {-0.050800000000010524, -0.04127499999988427}
      };

      ConvexPolygon2d poly = new ConvexPolygon2d(points);
      List<Point2d> polyPoints = new ArrayList<Point2d>();
      for (int i = 0; i < poly.getNumberOfVertices(); i++)
         polyPoints.add(poly.getVertex(i));
      
      boolean convexAndClockwise = InPlaceConvexHullCalculator2d.isConvexAndClockwise(polyPoints);

      for( int i = 0; i < points.length; i++ ) {
         assertTrue(poly.isPointInside(points[i][0],points[i][1]));
      }
      
      if (!convexAndClockwise)
         throw new RuntimeException("convexHull is not Convex and Clockwise!");

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testThreePointProblem()
   {
      ArrayList<Point2d> pointList = new ArrayList<Point2d>();

      Point2d point1 = new Point2d(2.0, 0.0);
      Point2d point2 = new Point2d(2.0, 2.0);
      Point2d point3 = new Point2d(0.0, 0.0);

      pointList.add(point1);
      pointList.add(point2);
      pointList.add(point3);

      @SuppressWarnings("unused")
      List<Point2d> convexHull = ConvexHullCalculator2d.getConvexHullCopy(pointList);

//    convexHull = ConvexHullCalculator2d.getConvexHullCopy(pointList);


//    for (int i=0; i<100000; i++)
//    {
////       System.out.println("i = " + i);
//       ArrayList<Point2d> convexHull = ConvexHullCalculator2d.getConvexHullCopy(pointList);
//    }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testEqualXProblem()
   {
      ArrayList<Point2d> pointList = new ArrayList<Point2d>();

      double[] xS = new double[]
      {
         0.1, 0.2, 0.3, 0.4, 0.5, 0.6
      };
      Random random = new Random(1776L);

      int numRandom = 10000;

      for (int i = 0; i < numRandom; i++)
      {
         int randomX = random.nextInt(xS.length);
         double x = xS[randomX];

         double y = random.nextDouble();

         Point2d point = new Point2d(x, y);

         pointList.add(point);

      }

      @SuppressWarnings("unused")
      List<Point2d> convexHull = ConvexHullCalculator2d.getConvexHullCopy(pointList);

//    convexHull = ConvexHullCalculator2d.getConvexHullCopy(pointList);


//    for (int i=0; i<100000; i++)
//    {
////       System.out.println("i = " + i);
//       ArrayList<Point2d> convexHull = ConvexHullCalculator2d.getConvexHullCopy(pointList);
//    }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetUpperHull()
   {
      ArrayList<Point2d> coincidalPointsRet = new ArrayList<Point2d>();
      ConvexHullCalculator2d.getUpperHull(coincidalPointsRet, coincidalPoints);
      assertEquals("Coincidal points not handled properly", coincidalPoints.get(0).getX(), coincidalPointsRet.get(0).getX(), 1e-7);
      assertEquals("Coincidal points not handled properly", coincidalPoints.get(0).getY(), coincidalPointsRet.get(0).getY(), 1e-7);

      ArrayList<Point2d> twoPointsEqualXRet = new ArrayList<Point2d>();
      ConvexHullCalculator2d.getUpperHull(twoPointsEqualXRet, twoPointsEqualX);
      assertEquals("2 points with equal x-coordinates not handled correctly", twoPointsEqualX.get(1).getY(), twoPointsEqualXRet.get(0).getY(), 1e-7);

      ArrayList<Point2d> fourPointsRet = new ArrayList<Point2d>();
      ConvexHullCalculator2d.getUpperHull(fourPointsRet, fourPoints);
      assertEquals("Size of returned list not correct for four points case", 3, fourPointsRet.size());
      assertTrue("fourPointsRet list is not convex", ConvexHullCalculator2d.isConvexAndClockwise(fourPointsRet));

      ArrayList<Point2d> eightPointsRet = new ArrayList<Point2d>();
      ConvexHullCalculator2d.getUpperHull(eightPointsRet, eightPoints);
      assertEquals("Size of returned list not correct for eight points case", 4, eightPointsRet.size());
      assertTrue("eightPointsRet is not convex", ConvexHullCalculator2d.isConvexAndClockwise(eightPointsRet));

      for (int numPoints = 3; numPoints < RANDOMLISTSIZEMAX; numPoints++)
      {
         randomList.clear();

         for (int j = 0; j < numPoints; j++)
         {
            randomList.add(new Point2d(random.nextDouble(), random.nextDouble()));
         }

         ArrayList<Point2d> randomListRet = new ArrayList<Point2d>();
         ConvexHullCalculator2d.getUpperHull(randomListRet, randomList);
         assertTrue("randomListRet is not convex", ConvexHullCalculator2d.isConvexAndClockwise(randomListRet));
      }

      /*
       *   TODO: write better tests (maybe).
       *   Passing the tests is a necessary but not sufficient condition
       *   The non-random cases have been verified manually though.
       */
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetLowerHull()
   {
      ArrayList<Point2d> coincidalPointsRet = new ArrayList<Point2d>();
      ConvexHullCalculator2d.getLowerHull(coincidalPointsRet, coincidalPoints);
      
      assertEquals("Coincidal points not handled properly", coincidalPoints.get(0).getX(), coincidalPointsRet.get(0).getX(), 1e-7);
      assertEquals("Coincidal points not handled properly", coincidalPoints.get(0).getY(), coincidalPointsRet.get(0).getY(), 1e-7);

      ArrayList<Point2d> twoPointsEqualXRet = new ArrayList<Point2d>();
      ConvexHullCalculator2d.getLowerHull(twoPointsEqualXRet, twoPointsEqualX);
      assertEquals("2 points with equal x-coordinates not handled correctly", twoPointsEqualX.get(0).getY(), twoPointsEqualXRet.get(0).getY(), 1e-7);

      ArrayList<Point2d> fourPointsRet = new ArrayList<Point2d>();
      ConvexHullCalculator2d.getLowerHull(fourPointsRet, fourPoints);
      assertEquals("Size of returned list not correct for four points case", 3, fourPointsRet.size());
      assertTrue("fourPointsRet list is not convex", ConvexHullCalculator2d.isConvexAndClockwise(fourPointsRet));

      ArrayList<Point2d> eightPointsRet = new ArrayList<Point2d>();
      ConvexHullCalculator2d.getLowerHull(eightPointsRet, eightPoints);
      assertEquals("Size of returned list not correct for eight points case", 3, eightPointsRet.size());
      assertTrue("eightPointsRet is not convex", ConvexHullCalculator2d.isConvexAndClockwise(eightPointsRet));

      for (int numPoints = 3; numPoints < RANDOMLISTSIZEMAX; numPoints++)
      {
         randomList.clear();

         for (int j = 0; j < numPoints; j++)
         {
            randomList.add(new Point2d(random.nextDouble(), random.nextDouble()));
         }

         ArrayList<Point2d> randomListRet = new ArrayList<Point2d>();
         ConvexHullCalculator2d.getUpperHull(randomListRet, randomList);
         assertTrue("randomListRet is not convex", ConvexHullCalculator2d.isConvexAndClockwise(randomListRet));
      }


      /*
       *   TODO: write better tests (maybe).
       *   Passing the tests is a necessary but not sufficient condition
       *   The non-random cases have been verified manually though.
       */
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGetConvexHull()
   {
      ArrayList<Point2d> coincidalPointsRet = new ArrayList<Point2d>();
      ConvexHullCalculator2d.getConvexHull(coincidalPointsRet, coincidalPoints);
      assertEquals("Coincidal points not handled properly", coincidalPoints.get(0).getX(), coincidalPointsRet.get(0).getX(), 1e-7);
      assertEquals("Coincidal points not handled properly", coincidalPoints.get(0).getY(), coincidalPointsRet.get(0).getY(), 1e-7);
      assertEquals("Size of returned list not correct for 2 coincidal points case", 1, coincidalPointsRet.size());
      boolean coincidalPointsTestBoolean = coincidalPointsRet.get(0).equals(coincidalPoints.get(0)) || coincidalPointsRet.get(0).equals(coincidalPoints.get(1));
      assertTrue("Reference to returned point is not the same as reference to either input point", coincidalPointsTestBoolean);

      ArrayList<Point2d> twoPointsEqualXRet = new ArrayList<Point2d>();
      ConvexHullCalculator2d.getConvexHull(twoPointsEqualXRet, twoPointsEqualX);
      assertEquals("Size of returned list not correct for 2 points equal x case", 2, twoPointsEqualXRet.size());

      ArrayList<Point2d> fourPointsRet = new ArrayList<Point2d>();
      ConvexHullCalculator2d.getConvexHull(fourPointsRet, fourPoints);
      assertEquals("Size of returned list not correct for four points case", 4, fourPointsRet.size());
      assertTrue("fourPointsRet list is not convex", ConvexHullCalculator2d.isConvexAndClockwise(fourPointsRet));

      ArrayList<Point2d> eightPointsRet = new ArrayList<Point2d>();
      ConvexHullCalculator2d.getConvexHull(eightPointsRet, eightPoints);
      assertEquals("Size of returned list not correct for eight points case", 6, eightPointsRet.size());
      assertTrue("eightPointsRet is not convex", ConvexHullCalculator2d.isConvexAndClockwise(eightPointsRet));

      for (int numPoints = 3; numPoints < RANDOMLISTSIZEMAX; numPoints++)
      {
         randomList.clear();

         for (int j = 0; j < numPoints; j++)
         {
            randomList.add(new Point2d(random.nextDouble(), random.nextDouble()));
         }

         ArrayList<Point2d> randomListRet = new ArrayList<Point2d>();
         ConvexHullCalculator2d.getConvexHull(randomListRet, randomList);
         assertTrue("randomListRet is not convex", ConvexHullCalculator2d.isConvexAndClockwise(randomListRet));
      }

      /*
       *    TODO: write better tests (maybe).
       *    Passing the tests is a necessary but not sufficient condition
       *    The non-random cases have been verified manually though.
       */
   }

   // TODO: write test for isConvex(.)
}
