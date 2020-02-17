package us.ihmc.robotics.functionApproximation;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class GradientDescentLinearRegressionTest
{
   private static final double epsilon = 1e-5;
   @Test
   public void testPerfectFunction()
   {

      GradientDescentLinearRegression linearRegression = new GradientDescentLinearRegression();
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
         }

         for (int value= 0; value < valuesToUse; value++)
         {
            double x = RandomNumbers.nextDouble(random, 10.0);
            assertEquals(a0 + a1 * x, linearRegression.computeY(x), epsilon);
         }
      }
   }
}
