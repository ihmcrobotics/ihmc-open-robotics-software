package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.assertEquals;

import javax.vecmath.Point2d;

import org.junit.Test;

public class ThreePointDoubleSplines1DTest
{
   
   @Test
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
   }
   
   
   @Test
   public void testPerfectLineExample()
   {
      ThreePointDoubleSplines1D spline = new ThreePointDoubleSplines1D();

      double zHeight = 0.23;
      double slope = 1.2;
      
      Point2d pointOne = new Point2d(0.0, zHeight);
      Point2d pointTwo = new Point2d(1.0, zHeight + 1.0 * slope);
      Point2d pointThree = new Point2d(2.0, zHeight + 2.0 * slope);

      Point2d[] points = new Point2d[] {pointOne, pointTwo, pointThree};
      double[] slopes = new double[] {1.1, 1.1, 1.1};
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

  
   
   
}
