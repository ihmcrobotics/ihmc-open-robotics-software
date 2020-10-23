package us.ihmc.robotics.functionApproximation;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.robotics.statistics.OnlineStandardDeviationCalculator;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class OnlineLeastSquaresRegressionTest
{
   private static final double epsilon = 1e-5;

   @Test
   public void testPerfectFunction()
   {

      OnlineLeastSquaresRegression linearRegression = new OnlineLeastSquaresRegression("", new YoRegistry("testRegistry"));
      OnlineStandardDeviationCalculator residualStats = new OnlineStandardDeviationCalculator("", new YoRegistry("testRegistry"));
      int functionsToEstimate = 10;
      int valuesToUse = 200;
      Random random = new Random(1738L);
      for (int function = 0; function < functionsToEstimate; function++)
      {
         linearRegression.reset();
         double a0 = RandomNumbers.nextDouble(random, 10);
         double a1 = RandomNumbers.nextDouble(random, 10);
         for (int value = 0; value < valuesToUse; value++)
         {
            double x = RandomNumbers.nextDouble(random, 10.0);
            double y = a0 + a1 * x;
            linearRegression.update(x, y);

            double residual = y - linearRegression.computeY(x);
            if (!Double.isNaN(residual) && value > 0 )
               residualStats.update(residual);
         }

         for (int value= 0; value < valuesToUse; value++)
         {
            double x = RandomNumbers.nextDouble(random, 10.0);
            assertEquals(a0 + a1 * x, linearRegression.computeY(x), epsilon);
         }
         assertEquals(1.0, linearRegression.getRSquared(), epsilon);
         assertEquals(0.0, residualStats.getStandardDeviation(), epsilon);
         assertEquals(0.0, residualStats.getVariance(), epsilon);
      }
   }
}
