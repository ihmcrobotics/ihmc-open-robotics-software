package us.ihmc.robotics.math.filters;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class FilteredVelocityYoVariableTest
{

   private static final double DT = 0.1;

	@Test
   public void testUpdateForTranslationalVelocity()
   {
      YoRegistry registry = new YoRegistry("testRegistry");
      double alpha = 0.3;
      YoDouble positionVariable = new YoDouble("positionVariable", registry);

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

	@Test
   public void testUpdateForRotationalVelocity()
   {
      YoRegistry registry = new YoRegistry("testRegistry");
      double alpha = 0.005;
      YoDouble positionVariable = new YoDouble("positionVariable", registry);

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
