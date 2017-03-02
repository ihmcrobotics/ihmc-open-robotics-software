package us.ihmc.robotics.math.filters;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class AlphaBetaFilteredYoVariableTest
{
   private static final double DT = 0.1;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testAlphaBetaFilteredVelocityAndPositionEstimatesWithNoVelocity()
   {
      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      DoubleYoVariable positionVariable = new DoubleYoVariable("positionVariable", registry);
      DoubleYoVariable xMeasuredVariable = new DoubleYoVariable("xMeasuredVariable", registry);

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

	@ContinuousIntegrationTest(estimatedDuration = 1.9)
	@Test(timeout=300000)
   public void testAlphaBetaFilteredVelocityAndPositionEstimatesWithConstantVelocity()
   {
      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      DoubleYoVariable positionVariable = new DoubleYoVariable("positionVariable", registry);
      DoubleYoVariable xMeasuredVariable = new DoubleYoVariable("xMeasuredVariable", registry);

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
