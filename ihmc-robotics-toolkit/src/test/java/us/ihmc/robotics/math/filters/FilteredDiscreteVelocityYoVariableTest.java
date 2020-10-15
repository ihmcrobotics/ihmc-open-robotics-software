package us.ihmc.robotics.math.filters;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class FilteredDiscreteVelocityYoVariableTest
{

   private static final double DT = 0.1;

	@Test
   public void testFilteredDiscreteVelocityNoDirectionChange()
   {
      double alpha = 0.99;
      YoRegistry registry = new YoRegistry("testRegistry");
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

	@Test
   public void testFilteredDiscreteVelocityWithDirectionChange()
   {
      double alpha = 0.99;
      YoRegistry registry = new YoRegistry("testRegistry");
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
