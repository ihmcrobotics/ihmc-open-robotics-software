package us.ihmc.commonWalkingControlModules.trajectories;

import javax.vecmath.Point2d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.trajectories.YoPolynomial;


public class TwoPointSpline1D
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private static final int numberOfCoefficients = 6;

   private final Point2d[] points = new Point2d[2];
   private double[] slopes = new double[2];
   private double[] secondDerivatives = new double[2];
   private YoPolynomial spline = new YoPolynomial("heightSpline", numberOfCoefficients, registry);

   public TwoPointSpline1D(YoVariableRegistry parentRegistry)
   {
      for (int i = 0; i < points.length; i++)
      {
         points[i] = new Point2d();
      }
      parentRegistry.addChild(registry);
   }

   public void setPoints(Point2d[] points, double[] slopes, double[] secondDerivatives)
   {
      for (int i = 0; i < points.length; i++)
      {
         this.points[i].set(points[i]);
         this.slopes[i] = slopes[i];
         this.secondDerivatives[i] = secondDerivatives[i];
      }

      spline.setQuintic(points[0].x, points[1].x, points[0].y, slopes[0], secondDerivatives[0], points[1].y, slopes[1], secondDerivatives[1]);
   }

   public double[] getZSlopeAndSecondDerivative(double queryPoint)
   {
      spline.compute(queryPoint);
      double z = spline.getPosition();
      double slope = spline.getVelocity();
      double secondDerivative = spline.getAcceleration();

      return new double[] {z, slope, secondDerivative};
   }
}
