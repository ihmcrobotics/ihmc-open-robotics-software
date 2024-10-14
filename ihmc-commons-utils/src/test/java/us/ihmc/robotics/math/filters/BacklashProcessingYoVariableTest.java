package us.ihmc.robotics.math.filters;


import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import static org.junit.jupiter.api.Assertions.*;

public class BacklashProcessingYoVariableTest
{
   @Test
   public void testAgainstRevisedBacklash()
   {
      YoRegistry registry = new YoRegistry("dummy");
      YoDouble slopTime = new YoDouble("slopTime", registry);
      double dt = 0.002;
      YoDouble alpha = new YoDouble("alpha", registry);
      alpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(16.0, dt));
      YoDouble positionVariable = new YoDouble("rawPosition", registry);
      FilteredFiniteDifferenceYoVariable velocityVariable = new FilteredFiniteDifferenceYoVariable("fd", "", alpha, positionVariable, dt, registry);

      BacklashProcessingYoVariable blToTest = new BacklashProcessingYoVariable("blTest", "", velocityVariable, dt, slopTime, registry);
      
      BacklashCompensatingVelocityYoVariable blExpected = new BacklashCompensatingVelocityYoVariable("blExpected", "", alpha, positionVariable, dt, slopTime, registry);

      Random random = new Random(561651L);

      for (double t = 0.0; t < 100.0; t += dt)
      {
         positionVariable.set(2.0 * Math.sin(2.0 * Math.PI * 10.0) + RandomNumbers.nextDouble(random, 1.0) * Math.sin(2.0 * Math.PI * 30.0 + 2.0 / 3.0 * Math.PI));
         
         velocityVariable.update();
         
         blToTest.update();
         blExpected.update();
         
         assertEquals(blToTest.getDoubleValue(), blExpected.getDoubleValue(), 1.0e-10);
      }
   }

}
