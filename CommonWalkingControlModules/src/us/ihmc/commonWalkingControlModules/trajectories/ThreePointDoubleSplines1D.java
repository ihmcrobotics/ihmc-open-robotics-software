package us.ihmc.commonWalkingControlModules.trajectories;

import javax.vecmath.Point2d;

public class ThreePointDoubleSplines1D
{
   private final Point2d[] points = new Point2d[3]; 
   private double[] slopes = new double[3];
   private double[] secondDerivatives = new double[3];
   
   public ThreePointDoubleSplines1D()
   {
      for (int i=0; i<points.length; i++)
      {
         points[i] = new Point2d();
      }
   }

   public void setPoints(Point2d[] points, double[] slopes, double[] secondDerivatives)
   {
      for (int i=0; i<points.length; i++)
      {
         this.points[i].set(points[i]);
         this.slopes[i] = slopes[i];
         this.secondDerivatives[i] = secondDerivatives[i];
      }
   }

   public double[] getZSlopeAndSecondDerivative(double queryPoint)
   {
      double z = points[0].getY();
      double slope = 0.0;
      double secondDerivative = 0.0;
      
      return new double[]{z, slope, secondDerivative};
   }

}
