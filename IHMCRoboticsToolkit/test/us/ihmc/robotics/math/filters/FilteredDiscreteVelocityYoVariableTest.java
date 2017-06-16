package us.ihmc.robotics.math.filters;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class FilteredDiscreteVelocityYoVariableTest
{

   private static final double DT = 0.1;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testFilteredDiscreteVelocityNoDirectionChange()
   {
      double alpha = 0.99;
      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      YoDouble positionVariable = new YoDouble("positionVariable", registry);
      YoDouble time = new YoDouble("time", registry);
      FilteredDiscreteVelocityYoVariable filteredDiscreteVelocityYoVariable = new FilteredDiscreteVelocityYoVariable("filteredDiscreteVelocityYoVariable", "",
            alpha, positionVariable, time, registry);

      positionVariable.set(10);
      time.set(0);

      for (int i = 0; i < 1000 / DT; i++)
      {
         time.add(DT);
         positionVariable.add(1);
         filteredDiscreteVelocityYoVariable.update();
      }

      assertEquals(10, filteredDiscreteVelocityYoVariable.getDoubleValue(), 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testFilteredDiscreteVelocityWithDirectionChange()
   {
      double alpha = 0.99;
      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      YoDouble positionVariable = new YoDouble("positionVariable", registry);
      YoDouble time = new YoDouble("time", registry);
      FilteredDiscreteVelocityYoVariable filteredDiscreteVelocityYoVariable = new FilteredDiscreteVelocityYoVariable("filteredDiscreteVelocityYoVariable", "",
            alpha, positionVariable, time, registry);

      positionVariable.set(10);
      time.set(0);

      for (int i = 0; i < 1000 / DT; i++)
      {
         time.add(DT);
         positionVariable.add(1);
         filteredDiscreteVelocityYoVariable.update();
      }

      assertEquals(10, filteredDiscreteVelocityYoVariable.getDoubleValue(), 1e-7);

      for (int i = 0; i < 1000 / DT; i++)
      {
         time.add(DT);
         positionVariable.add(-1);
         filteredDiscreteVelocityYoVariable.update();
      }

      assertEquals(-10, filteredDiscreteVelocityYoVariable.getDoubleValue(), 1e-7);
   }

}
