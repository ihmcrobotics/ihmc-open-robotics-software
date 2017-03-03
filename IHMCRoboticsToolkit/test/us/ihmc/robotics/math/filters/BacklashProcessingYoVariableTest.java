package us.ihmc.robotics.math.filters;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class BacklashProcessingYoVariableTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testAgainstRevisedBacklash()
   {
      YoVariableRegistry registry = new YoVariableRegistry("dummy");
      DoubleYoVariable slopTime = new DoubleYoVariable("slopTime", registry);
      double dt = 0.002;
      DoubleYoVariable alpha = new DoubleYoVariable("alpha", registry);
      alpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(16.0, dt));
      DoubleYoVariable positionVariable = new DoubleYoVariable("rawPosition", registry);
      FilteredVelocityYoVariable velocityVariable = new FilteredVelocityYoVariable("fd", "", alpha, positionVariable, dt, registry);

      BacklashProcessingYoVariable blToTest = new BacklashProcessingYoVariable("blTest", "", velocityVariable, dt, slopTime, registry);
      
      RevisedBacklashCompensatingVelocityYoVariable blExpected = new RevisedBacklashCompensatingVelocityYoVariable("blExpected", "", alpha, positionVariable, dt, slopTime, registry);

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
