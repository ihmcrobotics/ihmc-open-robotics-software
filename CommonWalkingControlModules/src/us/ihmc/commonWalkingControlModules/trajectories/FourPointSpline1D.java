package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.trajectories.YoPolynomial;

public class FourPointSpline1D
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private static final int numberOfCoefficients = 4; //10;

   private final Point2D[] points = new Point2D[4];
   private double[] endpointSlopes = new double[2];
   private double[] intermediateSlopes = new double[2];
   private YoPolynomial spline = new YoPolynomial("heightSpline", numberOfCoefficients, registry);

   public FourPointSpline1D(YoVariableRegistry parentRegistry)
   {
      for (int i = 0; i < points.length; i++)
      {
         points[i] = new Point2D();
      }
      parentRegistry.addChild(registry);
   }

   public void setPoints(Point2D[] points, double[] endpointSlopes, double[] intermediateSlopes)
   {
      // Assumes x values are increasing order:

      double previousX = Double.NEGATIVE_INFINITY;
      for (int i = 0; i < points.length; i++)
      {
         if (points[i].getX() < previousX)
            throw new RuntimeException("x values must be in increasing order!");
         this.points[i].set(points[i]);
         previousX = points[i].getX();
      }

      for (int i = 0; i < endpointSlopes.length; i++)
      {
         this.endpointSlopes[i] = endpointSlopes[i];
         this.intermediateSlopes[i] = intermediateSlopes[i];
      }

      spline.setCubicUsingIntermediatePoints(points[0].getX(), points[1].getX(), points[2].getX(), points[3].getX(), points[0].getY(), points[1].getY(), points[2].getY(), points[3].getY());

      //    spline.setNonic(points[0].x, points[1].x, points[2].x, points[3].x, points[0].y, endpointSlopes[0],
      //    points[1].y, intermediateSlopes[0], points[2].y, intermediateSlopes[1], points[3].y, endpointSlopes[1]);

      //      spline.setSeptic(points[0].x, points[1].x, points[2].x, points[3].x, points[0].y, endpointSlopes[0],
      //            points[1].y, intermediateSlopes[0], points[2].y, intermediateSlopes[1], points[3].y, endpointSlopes[1]);
   }

   public void getZSlopeAndSecondDerivative(double queryPoint, double[] resultToPack)
   {
      spline.compute(queryPoint);
      double z = spline.getPosition();
      double slope = spline.getVelocity();
      double secondDerivative = spline.getAcceleration();

      if (queryPoint < points[0].getX() + 1e-3)
      {
         slope = 0.0;
         secondDerivative = 0.0;
      }

      else if (queryPoint > points[points.length - 1].getX() - 1e-3)
      {
         slope = 0.0;
         secondDerivative = 0.0;
      }

      resultToPack[0] = z;
      resultToPack[1] = slope;
      resultToPack[2] = secondDerivative;
   }
}
