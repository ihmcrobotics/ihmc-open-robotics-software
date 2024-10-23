package us.ihmc.yoVariables.filters;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import static org.junit.jupiter.api.Assertions.*;

public class AlphaFilteredWrappingYoVariableTest
{
   private static final boolean DEBUG = false;
   private final Random random = new Random();

   @Test
   public void testInputModulo()
   {
      YoRegistry registry = new YoRegistry("testRegistry");
      YoDouble alpha = new YoDouble("alpha", registry);
      alpha.set(0.0); //sets the alpha to 0.0 so that the correction is instantaneous to check the result which is in fact the input with modulo
      
      YoDouble positionVariable = new YoDouble("positionVariable", registry);
      AlphaFilteredWrappingYoVariable alphaFilteredWrappingYoVariable = new AlphaFilteredWrappingYoVariable("alphaFilteredWrappingYoVariable", "", registry, positionVariable, alpha, -2.0, 8.0);

      //test at the boundaries
      positionVariable.set(8.0);
      alphaFilteredWrappingYoVariable.update();
      assertTrue(MathTools.epsilonEquals(alphaFilteredWrappingYoVariable.getDoubleValue(), -2.0, 1e-10));
      
      positionVariable.set(-2.0);
      alphaFilteredWrappingYoVariable.update();
      assertTrue(MathTools.epsilonEquals(alphaFilteredWrappingYoVariable.getDoubleValue(), -2.0, 1e-10));

      //test when the input is over the upperLimit
      positionVariable.set(33.0);
      alphaFilteredWrappingYoVariable.update();
      assertTrue(MathTools.epsilonEquals(alphaFilteredWrappingYoVariable.getDoubleValue(), 3.0, 1e-10));
      
      positionVariable.set(38.0);
      alphaFilteredWrappingYoVariable.update();
      assertTrue(MathTools.epsilonEquals(alphaFilteredWrappingYoVariable.getDoubleValue(), -2.0, 1e-10));
      
      positionVariable.set(42.0);
      alphaFilteredWrappingYoVariable.update();
      assertTrue(MathTools.epsilonEquals(alphaFilteredWrappingYoVariable.getDoubleValue(), 2.0, 1e-10));
      
      //test when the input is under the lowerLimit
      positionVariable.set(-22.0);
      alphaFilteredWrappingYoVariable.update();
      assertTrue(MathTools.epsilonEquals(alphaFilteredWrappingYoVariable.getDoubleValue(), -2.0, 1e-10));
      
      positionVariable.set(-23.5);
      alphaFilteredWrappingYoVariable.update();
      assertTrue(MathTools.epsilonEquals(alphaFilteredWrappingYoVariable.getDoubleValue(), 6.5, 1e-10));

      positionVariable.set(-42.0);
      alphaFilteredWrappingYoVariable.update();
      assertTrue(MathTools.epsilonEquals(alphaFilteredWrappingYoVariable.getDoubleValue(), -2.0, 1e-10));
   }
   
   
	@Test
   public void testNoisyFixedPosition()
   {
      // Use a reasonably large alpha for a reasonably large amount of noise
      YoRegistry registry = new YoRegistry("testRegistry");
      YoDouble alpha = new YoDouble("alpha", registry);
      alpha.set(0.8);
      
      YoDouble positionVariable = new YoDouble("positionVariable", registry);
      AlphaFilteredWrappingYoVariable alphaFilteredWrappingYoVariable = new AlphaFilteredWrappingYoVariable("alphaFilteredWrappingYoVariable", "", registry, positionVariable, alpha, 0.0, 20.0);

      double pseudoNoise = 0;

      positionVariable.set(10.0);
      for (int i = 0; i < 10000; i++)
      {
         // Oscillate the position about some uniformly distributed fixed point slightly larger than 10
         if (i % 2 == 0)
         {
            pseudoNoise = random.nextDouble();
         }
         positionVariable.add(Math.pow(-1, i) * pseudoNoise);
         alphaFilteredWrappingYoVariable.update();
      }

      assertEquals(10.0, alphaFilteredWrappingYoVariable.getDoubleValue(), 1.0);
   }
	
	@Test
	public void testErrorAlwaysDecreases()
	{
	   // Use a reasonably large alpha for a reasonably large amount of noise
	   YoRegistry registry = new YoRegistry("testRegistry");
	   YoDouble alpha = new YoDouble("alpha", registry);
	   alpha.set(0.999999);

	   YoDouble positionVariable = new YoDouble("positionVariable", registry);
	   double lowerLimit = RandomNumbers.nextDouble(random, -100.0, 100.0);
      double upperLimit = RandomNumbers.nextDouble(random, -100.0, 100.0);
      if(upperLimit < lowerLimit)
      {
         double temp = lowerLimit;
         lowerLimit = upperLimit;
         upperLimit = temp;
      }
      
      AlphaFilteredWrappingYoVariable alphaFilteredWrappingYoVariable = new AlphaFilteredWrappingYoVariable("alphaFilteredWrappingYoVariable", "", registry, positionVariable, alpha, lowerLimit, upperLimit);
	   positionVariable.set(RandomNumbers.nextDouble(random, lowerLimit, upperLimit));
	   alphaFilteredWrappingYoVariable.update();
	   
	   for(int iteration = 0; iteration < 10000; iteration++)
	   {
	      positionVariable.set(RandomNumbers.nextDouble(random, lowerLimit, upperLimit));
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
	
   @Test
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
	   
   }

	@Test
   public void testAlphaAndBreakFrequencyComputations()
   {
      double DT = 0.1;
      double randomAlpha = random.nextDouble();
      double computedBreakFrequency = AlphaFilteredWrappingYoVariable.computeBreakFrequencyGivenAlpha(randomAlpha, DT);
      double computedAlpha = AlphaFilteredWrappingYoVariable.computeAlphaGivenBreakFrequencyProperly(computedBreakFrequency, DT);

      assertEquals(randomAlpha, computedAlpha, 1e-7);
      assertEquals(computedBreakFrequency, AlphaFilteredWrappingYoVariable.computeBreakFrequencyGivenAlpha(computedAlpha, DT), 1e-7);
      
      if(DEBUG)
      {
         System.out.println("Random Alpha: " + randomAlpha);
         System.out.println("Computed Alpha: " + AlphaFilteredWrappingYoVariable.computeAlphaGivenBreakFrequencyProperly(computedBreakFrequency, DT));
      }
   }
}
