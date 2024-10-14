package us.ihmc.robotics.math.filters;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class MovingAverageYoDoubleTest
{
   private final Random rng = new Random();

	@Test
   public void testBetaFilteredYoVariable()
   {
      int beta = 5000;
      double pseudoNoise = 0;

      YoRegistry registry = new YoRegistry("testRegistry");
      YoDouble positionVariable = new YoDouble("positionVariable", registry);
      MovingAverageYoDouble betaFilteredYoVariable = new MovingAverageYoDouble("betaFilteredYoVariable", registry, beta, positionVariable);

      positionVariable.set(10);

      for (int i = 0; i < 10000; i++)
      {
         if (i % 2 == 0)
         {
            pseudoNoise = rng.nextDouble();
         }
         positionVariable.add(Math.pow(-1, i) * pseudoNoise);
         betaFilteredYoVariable.update();
      }

      assertEquals(10, betaFilteredYoVariable.getDoubleValue(), 1);
   }

   @Test
   public void testTrueMovingAverage()
   {
      int beta = 10;

      YoRegistry registry = new YoRegistry("testRegistry");
      MovingAverageYoDouble betaFilteredYoVariable = new MovingAverageYoDouble("betaFilteredYoVariable", registry, beta);

      double epsilon = 1e-10;

      betaFilteredYoVariable.update(1.0);
      assertEquals(1.0, betaFilteredYoVariable.getDoubleValue(), epsilon);

      betaFilteredYoVariable.update(2.0);
      assertEquals(1.5, betaFilteredYoVariable.getDoubleValue(), epsilon);


      betaFilteredYoVariable.update(3.0);
      betaFilteredYoVariable.update(4.0);
      betaFilteredYoVariable.update(5.0);
      betaFilteredYoVariable.update(6.0);
      betaFilteredYoVariable.update(7.0);
      betaFilteredYoVariable.update(8.0);
      betaFilteredYoVariable.update(9.0);
      betaFilteredYoVariable.update(10.0);

      assertEquals(5.5, betaFilteredYoVariable.getDoubleValue(), epsilon);
   }


}
