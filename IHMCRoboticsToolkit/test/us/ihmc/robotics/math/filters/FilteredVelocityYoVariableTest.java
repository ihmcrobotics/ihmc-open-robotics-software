package us.ihmc.robotics.math.filters;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class FilteredVelocityYoVariableTest
{

   private static final double DT = 0.1;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testUpdateForTranslationalVelocity()
   {
      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      double alpha = 0.3;
      DoubleYoVariable positionVariable = new DoubleYoVariable("positionVariable", registry);

      FilteredVelocityYoVariable filteredVelocityYoVariable = new FilteredVelocityYoVariable("filteredVelocityYoVariable", "test description", alpha,
            positionVariable, DT, registry);

      filteredVelocityYoVariable.set(0);
      positionVariable.set(0);

      for (int i = 0; i < 10000; i++)
      {
         positionVariable.add(10);
         filteredVelocityYoVariable.update();
      }

      assertEquals(100, filteredVelocityYoVariable.getDoubleValue(), 1e-7);

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testUpdateForRotationalVelocity()
   {
      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      double alpha = 0.005;
      DoubleYoVariable positionVariable = new DoubleYoVariable("positionVariable", registry);

      FilteredVelocityYoVariable filteredVelocityYoVariable = new FilteredVelocityYoVariable("filteredVelocityYoVariable", "test description", alpha,
            positionVariable, DT, registry);

      filteredVelocityYoVariable.set(0);
      positionVariable.set(-Math.PI);

      for (int i = 0; i < 10000; i++)
      {
         if (positionVariable.getValueAsDouble() + 0.5 > Math.PI)
         {
            positionVariable.set(-Math.PI + (Math.PI - positionVariable.getValueAsDouble()));
         }
         else
         {
            positionVariable.add(.5);
         }

         filteredVelocityYoVariable.updateForAngles();
      }

      assertEquals(5, filteredVelocityYoVariable.getDoubleValue(), 1e-5);

   }
}
