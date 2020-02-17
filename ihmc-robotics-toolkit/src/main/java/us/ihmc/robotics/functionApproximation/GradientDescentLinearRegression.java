package us.ihmc.robotics.functionApproximation;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class GradientDescentLinearRegression
{
   private double b0 = 0.0;
   private double b1 = 0.0;
   private int numberOfIterations = 0;

   private double sumOfErrorSquared = 0.0;

   private double learningRate = 0.7;

   public void setLearningRate(double learningRate)
   {
      this.learningRate = learningRate;
   }

   public void reset()
   {
      b0 = 0.0;
      b1 = 0.0;
      sumOfErrorSquared = 0.0;
      numberOfIterations = 0;
   }

   public double computeY(double x)
   {
      return b0 + b1 * x;
   }

   public void update(Point2DReadOnly point)
   {
      update(point.getX(), point.getY());
   }

   public void update(double x, double y)
   {
      double predictedY = computeY(x);
      double error = predictedY - y;
      numberOfIterations++;

      double previousB0 = b0;
      double previousB1 = b1;

      b0 = b0 - learningRate / numberOfIterations * error;
      b1 = b1 - learningRate / numberOfIterations * error * x;

      sumOfErrorSquared += (y - (previousB1 * x + previousB0)) * (y - (b1 * x + b0 ));
   }

}
