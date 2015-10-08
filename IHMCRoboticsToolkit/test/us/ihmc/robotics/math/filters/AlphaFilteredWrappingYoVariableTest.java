package us.ihmc.robotics.math.filters;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class AlphaFilteredWrappingYoVariableTest
{
   private final Random random = new Random();

   //TODO: Test the modulo of the input
   
	@DeployableTestMethod(estimatedDuration = 0.1)
	@Test(timeout=60000)
   public void testNoisyFixedPosition()
   {
      // Use a reasonably large alpha for a reasonably large amount of noise
      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      DoubleYoVariable alpha = new DoubleYoVariable("alpha", registry);
      alpha.set(0.8);
      
      DoubleYoVariable positionVariable = new DoubleYoVariable("positionVariable", registry);
      AlphaFilteredWrappingYoVariable AlphaFilteredWrappingYoVariable = new AlphaFilteredWrappingYoVariable("AlphaFilteredWrappingYoVariable", "", registry, positionVariable, alpha, 0.0, 20.0);

      double pseudoNoise = 0;

      positionVariable.set(10);
      for (int i = 0; i < 10000; i++)
      {
         // Oscillate the position about some uniformly distributed fixed point slightly larger than 10
         if (i % 2 == 0)
         {
            pseudoNoise = random.nextDouble();
         }
         positionVariable.add(Math.pow(-1, i) * pseudoNoise);
         AlphaFilteredWrappingYoVariable.update();
      }

      assertEquals(10, AlphaFilteredWrappingYoVariable.getDoubleValue(), 1);
   }
	
	@DeployableTestMethod(estimatedDuration = 0.1)
	@Test(timeout=60000)
	public void testErrorAlwaysDecreases()
	{
	   // Use a reasonably large alpha for a reasonably large amount of noise
	   YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
	   DoubleYoVariable alpha = new DoubleYoVariable("alpha", registry);
	   alpha.set(0.999999);

	   DoubleYoVariable positionVariable = new DoubleYoVariable("positionVariable", registry);
	   double lowerLimit = RandomTools.generateRandomDouble(random, -100.0, 100.0);
      double upperLimit = RandomTools.generateRandomDouble(random, -100.0, 100.0);
      if(upperLimit < lowerLimit)
      {
         double temp = lowerLimit;
         lowerLimit = upperLimit;
         upperLimit = temp;
      }
      
      AlphaFilteredWrappingYoVariable alphaFilteredWrappingYoVariable = new AlphaFilteredWrappingYoVariable("AlphaFilteredWrappingYoVariable", "", registry, positionVariable, alpha, lowerLimit, upperLimit);
	   positionVariable.set(RandomTools.generateRandomDouble(random, lowerLimit, upperLimit));
	   alphaFilteredWrappingYoVariable.update();
	   
	   for(int iteration = 0; iteration < 10000; iteration++)
	   {
	      positionVariable.set(RandomTools.generateRandomDouble(random, lowerLimit, upperLimit));
	      double lastError = getErrorConsideringWrap(alphaFilteredWrappingYoVariable.getDoubleValue(), positionVariable.getDoubleValue(),lowerLimit, upperLimit);
	      for (int convergeAlphaCount = 0; convergeAlphaCount < 100; convergeAlphaCount++)
	      {
	         alphaFilteredWrappingYoVariable.update();
	         double currentError = getErrorConsideringWrap(alphaFilteredWrappingYoVariable.getDoubleValue(), positionVariable.getDoubleValue(),lowerLimit, upperLimit);
	         assertTrue(Math.abs(currentError) < Math.abs(lastError));
	      }
	   }
	}
	
	public double getErrorConsideringWrap(double current, double target, double lowerLimit, double upperLimit)
	{
	   double range = upperLimit - lowerLimit;
	   if(target > upperLimit)
	   {
	      target = (target - lowerLimit) % range + lowerLimit;
	   }
	   
	   if(target < lowerLimit)
	   {
	      double offset = (target - upperLimit) % range;
         target = offset + upperLimit;
	   }
	   
	   double standardError = target - current;
	   double wrappingError = 0.0;
	   if(target > current)
	   {
	      wrappingError = lowerLimit - current + target - upperLimit;
	   }
	   else
	   {
	      wrappingError = upperLimit - current + target - lowerLimit;
	   }
	   
	   if(Math.abs(standardError) < Math.abs(wrappingError))
	   {
	      return standardError;
	   }
	   return wrappingError;
	}
	
	@DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout=60000)
   public void testWrappingError()
   {
	   double e = getErrorConsideringWrap(0.2,0.8,0.0,1.0);
      assertEquals(-0.4, e, 0.001);
	   
	   e = getErrorConsideringWrap(0.8,0.2,0.0,1.0);
	   assertEquals(0.4, e, 0.001);

	   e = getErrorConsideringWrap(0.0,0.4,0,1);
	   assertEquals(0.4, e, 0.001);
	   
	   e = getErrorConsideringWrap(-0.2,0.4,-1.0,1.0);
	   assertEquals(0.6, e, 0.001);
	   
	   e = getErrorConsideringWrap(-1.0,1.0,-1.0,1.0);
	   assertEquals(0.0, e, 0.001);
	   
	   e = getErrorConsideringWrap(1.0,-1.0,-1.0,1.0);
	   assertEquals(0.0, e, 0.001);
	   
	   e = getErrorConsideringWrap(0.4, 1.6,-1.0,1.0);
	   assertEquals(-0.8, e, 0.001);
	   
	   e = getErrorConsideringWrap(-0.4, -1.6,-1.0,1.0);
	   assertEquals(0.8, e, 0.001);

	   e = getErrorConsideringWrap(0.4, -1.6,-1.0,1.0);
	   assertEquals(0.0, e, 0.001);
	   
	   e = getErrorConsideringWrap(0.2, 0.2,-1.0,1.0);
	   assertEquals(0.0, e, 0.001);
	   
	   e = getErrorConsideringWrap(-3.2, -4.0,-5.0,-1.0);
	   assertEquals(-0.8, e, 0.001);
	   
	   e = getErrorConsideringWrap(0.0, 0.0, -1.0, 1.0);
	   System.out.println(e);
	   
   }

	@DeployableTestMethod(estimatedDuration = 0.1)
	@Test(timeout=60000)
   public void testAlphaAndBreakFrequencyComputations()
   {
      double DT = 0.1;
      double randomAlpha = random.nextDouble();
      double computedBreakFrequency = AlphaFilteredWrappingYoVariable.computeBreakFrequencyGivenAlpha(randomAlpha, DT);
      double computedAlpha = AlphaFilteredWrappingYoVariable.computeAlphaGivenBreakFrequencyProperly(computedBreakFrequency, DT);

      assertEquals(randomAlpha, computedAlpha, 1e-7);
      assertEquals(computedBreakFrequency, AlphaFilteredWrappingYoVariable.computeBreakFrequencyGivenAlpha(computedAlpha, DT), 1e-7);

      System.out.println("Random Alpha: " + randomAlpha);
      System.out.println("Computed Alpha: " + AlphaFilteredWrappingYoVariable.computeAlphaGivenBreakFrequencyProperly(computedBreakFrequency, DT));

   }
}
