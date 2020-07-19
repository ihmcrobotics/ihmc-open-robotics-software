package us.ihmc.robotics.math.filters;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class AlphaBetaFilteredYoVariableTest
{
   private static final double DT = 0.1;

	@Test
   public void testAlphaBetaFilteredVelocityAndPositionEstimatesWithNoVelocity()
   {
      YoRegistry registry = new YoRegistry("testRegistry");
      YoDouble positionVariable = new YoDouble("positionVariable", registry);
      YoDouble xMeasuredVariable = new YoDouble("xMeasuredVariable", registry);

      double alpha = 0.2;
      double beta = 0.35;

      AlphaBetaFilteredYoVariable abFilteredYoVariable = new AlphaBetaFilteredYoVariable("abFilteredYoVariable", registry, alpha, beta, positionVariable,
            xMeasuredVariable, DT);

      abFilteredYoVariable.set(0);
      positionVariable.set(0);
      xMeasuredVariable.set(42);

      for (int i = 0; i < 10000; i++)
      {
         abFilteredYoVariable.update();
      }

      // Converges on 0 since position doesn't change (no dx/dt)
      assertEquals(0, abFilteredYoVariable.getDoubleValue(), 1e-7);

   }

	@Test
   public void testAlphaBetaFilteredVelocityAndPositionEstimatesWithConstantVelocity()
   {
      YoRegistry registry = new YoRegistry("testRegistry");
      YoDouble positionVariable = new YoDouble("positionVariable", registry);
      YoDouble xMeasuredVariable = new YoDouble("xMeasuredVariable", registry);

      double alpha = 0.2;
      double beta = 0.35;

      AlphaBetaFilteredYoVariable abFilteredYoVariable = new AlphaBetaFilteredYoVariable("abFilteredYoVariable", registry, alpha, beta, positionVariable,
            xMeasuredVariable, DT);

      for (int i = 0; i < 10000; i++)
      {
         abFilteredYoVariable.set(0);
         positionVariable.set(0);
         xMeasuredVariable.set(42);

         for (int j = 0; j < 10000; j++)
         {
            xMeasuredVariable.set(xMeasuredVariable.getDoubleValue() + 10); // Velocity = 100 distances per time
            abFilteredYoVariable.update();
         }

         assertEquals(100, abFilteredYoVariable.getDoubleValue(), 1e-7);
      }
   }

}
