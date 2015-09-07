package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.assertEquals;

import javax.vecmath.Point2d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;

public class ThreePointDoubleSplines1DTest
{
 
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
   @After
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

	@EstimatedDuration(duration = 0.3)
	@Test(timeout = 30000)
   public void testSimpleFlatExample()
   {
      ThreePointDoubleSplines1D spline = new ThreePointDoubleSplines1D();

      double zHeight = 0.23;

      Point2d pointOne = new Point2d(0.0, zHeight);
      Point2d pointTwo = new Point2d(1.0, zHeight);
      Point2d pointThree = new Point2d(2.0, zHeight);

      Point2d[] points = new Point2d[] {pointOne, pointTwo, pointThree};
      double[] slopes = new double[] {0.0, 0.0, 0.0};
      double[] secondDerivatives = new double[] {0.0, 0.0, 0.0};

      spline.setPoints(points, slopes, secondDerivatives);

      double queryPoint = 0.5;

      double[] zSlopeAndSecondDerivative = spline.getZSlopeAndSecondDerivative(queryPoint);
      double zAtQuery = zSlopeAndSecondDerivative[0];
      double slopeAtQuery = zSlopeAndSecondDerivative[1];
      double secondDerivativeAtQuery = zSlopeAndSecondDerivative[2];
      
      assertEquals(zHeight, zAtQuery, 1e-7);
      assertEquals(0.0, slopeAtQuery, 1e-7);
      assertEquals(0.0, secondDerivativeAtQuery, 1e-7);
   }

	@EstimatedDuration(duration = 0.4)
	@Test(timeout = 30000)
   public void testPerfectLineExample()
   {
      ThreePointDoubleSplines1D spline = new ThreePointDoubleSplines1D();

      double zHeight = 0.23;
      double slope = 1.2;
      
      Point2d pointOne = new Point2d(0.0, zHeight);
      Point2d pointTwo = new Point2d(1.0, zHeight + 1.0 * slope);
      Point2d pointThree = new Point2d(2.0, zHeight + 2.0 * slope);

      Point2d[] points = new Point2d[] {pointOne, pointTwo, pointThree};
      double[] slopes = new double[] {slope, slope, slope};
      double[] secondDerivatives = new double[] {0.0, 0.0, 0.0};

      spline.setPoints(points, slopes, secondDerivatives);

      double queryPoint = 0.5;

      double[] zSlopeAndSecondDerivative = spline.getZSlopeAndSecondDerivative(queryPoint);
      double zAtQuery = zSlopeAndSecondDerivative[0];
      double slopeAtQuery = zSlopeAndSecondDerivative[1];
      double secondDerivativeAtQuery = zSlopeAndSecondDerivative[2];

      assertEquals(zHeight + 0.5 * slope, zAtQuery, 1e-7);
      assertEquals(slope, slopeAtQuery, 1e-7);
      assertEquals(0.0, secondDerivativeAtQuery, 1e-7);
   }

	@EstimatedDuration(duration = 0.3)
	@Test(timeout = 30000)
   public void testFooExample()
   {
      double xAtOne = 0.2;
      double xAtTwo = 1.1;
      double xAtThree = 5.5;
      
      fooExample(xAtOne, xAtTwo, xAtThree);
   }

	@EstimatedDuration(duration = 0.3)
	@Test(timeout = 30000)
   public void testFooExamplePointsOutOfOrder()
   {
      double xAtOne = 5.5;
      double xAtTwo = 0.2;
      double xAtThree = 1.1;
      
      fooExample(xAtOne, xAtTwo, xAtThree);
   }


   private void fooExample(double xAtOne, double xAtTwo, double xAtThree)
   {
      ThreePointDoubleSplines1D spline = new ThreePointDoubleSplines1D();
      
      double heightAtOne = 0.23;
      double heightAtTwo = 0.9;
      double heightAtThree = 1.7;
      
      double slopeAtOne = 0.23;
      double slopeAtTwo = 0.9;
      double slopeAtThree = 1.7;
      
      double secondDerivativeAtOne = 0.23;
      double secondDerivativeAtTwo = 0.9;
      double secondDerivativeAtThree = 1.7;
      
      packSpline(spline, xAtOne, xAtTwo, xAtThree, heightAtOne, heightAtTwo, heightAtThree, slopeAtOne, slopeAtTwo, slopeAtThree, secondDerivativeAtOne,
            secondDerivativeAtTwo, secondDerivativeAtThree);
      
      double queryPoint = xAtOne;
      
      double[] zSlopeAndSecondDerivative = spline.getZSlopeAndSecondDerivative(queryPoint);
      double zAtQuery = zSlopeAndSecondDerivative[0];
      double slopeAtQuery = zSlopeAndSecondDerivative[1];
      double secondDerivativeAtQuery = zSlopeAndSecondDerivative[2];
      
      assertEquals(heightAtOne, zAtQuery, 1e-7);
      assertEquals(slopeAtOne, slopeAtQuery, 1e-7);
      assertEquals(secondDerivativeAtOne, secondDerivativeAtQuery, 1e-7);
      
      // At Two
      
      queryPoint = xAtTwo;
      
      zSlopeAndSecondDerivative = spline.getZSlopeAndSecondDerivative(queryPoint);
      zAtQuery = zSlopeAndSecondDerivative[0];
      slopeAtQuery = zSlopeAndSecondDerivative[1];
      secondDerivativeAtQuery = zSlopeAndSecondDerivative[2];
      
      assertEquals(heightAtTwo, zAtQuery, 1e-7);
      assertEquals(slopeAtTwo, slopeAtQuery, 1e-7);
      assertEquals(secondDerivativeAtTwo, secondDerivativeAtQuery, 1e-7);
      
      // At Three
      
      queryPoint = xAtThree;
      
      zSlopeAndSecondDerivative = spline.getZSlopeAndSecondDerivative(queryPoint);
      zAtQuery = zSlopeAndSecondDerivative[0];
      slopeAtQuery = zSlopeAndSecondDerivative[1];
      secondDerivativeAtQuery = zSlopeAndSecondDerivative[2];
      
      assertEquals(heightAtThree, zAtQuery, 1e-7);
      assertEquals(slopeAtThree, slopeAtQuery, 1e-7);
      assertEquals(secondDerivativeAtThree, secondDerivativeAtQuery, 1e-7);
   }

	@EstimatedDuration(duration = 0.4)
	@Test(timeout = 30000)
   public void testCompareOutOfOrderSplineWithNormalOne()
   {
      ThreePointDoubleSplines1D spline1 = new ThreePointDoubleSplines1D();
      ThreePointDoubleSplines1D spline2 = new ThreePointDoubleSplines1D();
      
      double xAtOne = 0.2;
      double xAtTwo = 1.1;
      double xAtThree = 5.5;
      
      double heightAtOne = 0.23;
      double heightAtTwo = 0.9;
      double heightAtThree = 1.7;
      
      double slopeAtOne = 0.23;
      double slopeAtTwo = 0.9;
      double slopeAtThree = 1.7;
      
      double secondDerivativeAtOne = 0.23;
      double secondDerivativeAtTwo = 0.9;
      double secondDerivativeAtThree = 1.7;
      
      packSpline(spline1, xAtOne, xAtTwo, xAtThree, heightAtOne, heightAtTwo, heightAtThree, slopeAtOne, slopeAtTwo, slopeAtThree, secondDerivativeAtOne,
            secondDerivativeAtTwo, secondDerivativeAtThree);
      
      packSpline(spline2, xAtTwo, xAtThree, xAtOne, heightAtTwo, heightAtThree, heightAtOne, slopeAtTwo, slopeAtThree, slopeAtOne, 
            secondDerivativeAtTwo, secondDerivativeAtThree, secondDerivativeAtOne);
      
      double queryPoint = 0.7;
      
      double[] zSlopeAndSecondDerivative1 = spline1.getZSlopeAndSecondDerivative(queryPoint);
      double zAtQuery1 = zSlopeAndSecondDerivative1[0];
      double slopeAtQuery1 = zSlopeAndSecondDerivative1[1];
      double secondDerivativeAtQuery1 = zSlopeAndSecondDerivative1[2];
      
      double[] zSlopeAndSecondDerivative2 = spline2.getZSlopeAndSecondDerivative(queryPoint);
      double zAtQuery2 = zSlopeAndSecondDerivative2[0];
      double slopeAtQuery2 = zSlopeAndSecondDerivative2[1];
      double secondDerivativeAtQuery2 = zSlopeAndSecondDerivative2[2];
      
      assertEquals(zAtQuery1, zAtQuery2, 1e-7);
      assertEquals(slopeAtQuery1, slopeAtQuery2, 1e-7);    
      assertEquals(secondDerivativeAtQuery1, secondDerivativeAtQuery2, 1e-7);    
   }


   private void packSpline(ThreePointDoubleSplines1D spline, double xAtOne, double xAtTwo, double xAtThree, double heightAtOne, double heightAtTwo,
         double heightAtThree, double slopeAtOne, double slopeAtTwo, double slopeAtThree, double secondDerivativeAtOne, double secondDerivativeAtTwo,
         double secondDerivativeAtThree)
   {
      Point2d pointOne = new Point2d(xAtOne, heightAtOne);
      Point2d pointTwo = new Point2d(xAtTwo, heightAtTwo);
      Point2d pointThree = new Point2d(xAtThree, heightAtThree);
      
      Point2d[] points = new Point2d[]{pointOne, pointTwo, pointThree};
      double[] slopes = new double[]{slopeAtOne, slopeAtTwo, slopeAtThree};
      double[] secondDerivatives = new double[]{secondDerivativeAtOne, secondDerivativeAtTwo, secondDerivativeAtThree};
      
      spline.setPoints(points, slopes, secondDerivatives);
   }
}
