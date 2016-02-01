package us.ihmc.robotics.numericalMethods;

public class NewtonRaphsonMethod
{
   private double previousX;
   private double previousY;

   private double change = Double.POSITIVE_INFINITY;

   private int iterations = 0;
   private final int maxIterations;

   private final double epsilon;

   public NewtonRaphsonMethod(int maxIterations, double epsilon)
   {
      this(maxIterations, epsilon, 0.0, Double.NaN);
   }

   public NewtonRaphsonMethod(int maxIterations, double epsilon, double initialX, double initialY)
   {
      doChecks(maxIterations, epsilon);

      this.maxIterations = maxIterations;
      this.epsilon = epsilon;

      previousX = initialX;
      previousY = initialY;
   }

   public void update(double x, double y)
   {
      double derivative;    // approximation of the derivative.
      if (Double.isNaN(previousY))
      {
         derivative = getRandomDerivative();
      }
      else
      {
         derivative = (y - previousY) / (x - previousX);
      }

      update(x, y, derivative);
   }

   public void update(double x, double y, double derivative)
   {
      change = -y / derivative;

      previousX = x;
      previousY = y;

      iterations++;
   }

   public double nextX()
   {
      return previousX + change;
   }

   public boolean stop()
   {
      if ((iterations >= maxIterations) || (Math.abs(previousY) < epsilon))
      {
         return true;
      }
      else
      {
         return false;
      }
   }

   public int getIterations()
   {
      return iterations;
   }

   private double getRandomDerivative()
   {
      return 1e-7;
   }

   private static void doChecks(int maxIterations, double epsilon)
   {
      // NaN:
      if (Double.isNaN(epsilon))
      {
         throw new RuntimeException("Double.isNaN(epsilon)");
      }

      // Nonnegative:
      if (maxIterations < 0)
      {
         throw new RuntimeException("maxIterations < 0");
      }

      if (epsilon < 0.0)
      {
         throw new RuntimeException("epsilon < 0.0");
      }
   }
}
