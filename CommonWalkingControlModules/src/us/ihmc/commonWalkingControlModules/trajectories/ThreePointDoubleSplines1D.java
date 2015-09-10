package us.ihmc.commonWalkingControlModules.trajectories;

import javax.vecmath.Point2d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.math.trajectories.YoPolynomial;


public class ThreePointDoubleSplines1D
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private static final int numberOfCoefficients = 6;

   private final Point2d[] points = new Point2d[3];
   private double[] slopes = new double[3];
   private double[] secondDerivatives = new double[3];
   private YoPolynomial splines[] = new YoPolynomial[2];

   public ThreePointDoubleSplines1D()
   {
      for (int i = 0; i < points.length; i++)
      {
         points[i] = new Point2d();
      }

      for (int i = 0; i < splines.length; i++)
      {
         splines[i] = new YoPolynomial("spline" + i, numberOfCoefficients, registry);
      }
   }

   public void setPoints(Point2d[] pointsIn, double[] slopes, double[] secondDerivatives)
   {
      for (int i = 0; i < points.length; i++)
      {
         this.points[i].set(pointsIn[i]);
         this.slopes[i] = slopes[i];
         this.secondDerivatives[i] = secondDerivatives[i];
      }

      reorderPoints(points, slopes, secondDerivatives);

      for (int i = 0; i < splines.length; i++)
      {
         splines[i].setQuintic(points[i].x, points[i + 1].x, points[i].y, slopes[i], secondDerivatives[i], points[i + 1].y, slopes[i + 1],
                               secondDerivatives[i + 1]);
      }
   }

   private void reorderPoints(Point2d[] points, double[] slopes, double[] secondDerivatives)
   {
      for (int i = 0; i < 2; i++)
      {
         for (int j = i + 1; j < 3; j++)
         {
            if (points[i].x > points[j].x)
            {
               swapPoints(points, i, j);
               swapDoubles(slopes, i, j);
               swapDoubles(secondDerivatives, i, j);
            }
         }
      }
   }

   private void swapPoints(Point2d[] points, int index1, int index2)
   {
      Point2d temp = null;
      temp = points[index1];
      points[index1] = points[index2];
      points[index2] = temp;
   }

   private void swapDoubles(double[] doubles, int index1, int index2)
   {
      double temp = Double.NaN;
      temp = doubles[index1];
      doubles[index1] = doubles[index2];
      doubles[index2] = temp;
   }

   public double[] getZSlopeAndSecondDerivative(double queryPoint)
   {
      for (int i = 0; i < splines.length; i++)
      {
         if (MathTools.isInsideBoundsInclusive(queryPoint, points[i].x, points[i + 1].x))
         {
            splines[i].compute(queryPoint);
            double z = splines[i].getPosition();
            double slope = splines[i].getVelocity();
            double secondDerivative = splines[i].getAcceleration();

            return new double[] {z, slope, secondDerivative};
         }
      }

      throw new RuntimeException("queryPoint out of range");
   }

}
