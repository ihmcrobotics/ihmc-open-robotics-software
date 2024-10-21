package us.ihmc.robotics.controllers.pidGains.implementations;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.wholeBodyControlCore.pidGains.GainCalculator;

public class GainCalculatorTest
{
   @Test
   public void testComputeDerivativeGain()
   {
      Random random = new Random();
      for (int i = 0; i < 1000; i++)
      {
         double derivativeGain = GainCalculator.computeDerivativeGain(random.nextDouble() * 100 * -1, random.nextDouble() * 100);
         assertTrue(Double.isNaN(derivativeGain));
         
         derivativeGain = GainCalculator.computeDerivativeGain(random.nextDouble() * 100, random.nextDouble() * 100 * -1);
         assertTrue(derivativeGain < 0.0);
      }
   }

   @Test
   public void testComputeDampingRatio()
   {
      Random random = new Random();
      for (int i = 0; i < 1000; i++)
      {
         double dampingRatio = GainCalculator.computeDampingRatio(random.nextDouble() * 100 * -1, random.nextDouble() * 100);
         assertTrue(Double.isNaN(dampingRatio));

         dampingRatio = GainCalculator.computeDerivativeGain(random.nextDouble() * 100, random.nextDouble() * 100 * -1);
         assertTrue(dampingRatio < 0.0);

         double proportionalGain = random.nextDouble() * 100;
         double derivativeGain = random.nextDouble() * 10;
         dampingRatio = GainCalculator.computeDampingRatio(proportionalGain, derivativeGain);
         assertEquals(derivativeGain / (2 * Math.sqrt(proportionalGain)), dampingRatio, 1e-4);
      }
   }

   @Test
   public void testComputeDampingForSecondOrderSystem()
   {
      double dampingCoeff = GainCalculator.computeDampingForSecondOrderSystem(1, 1, 1);
      assertEquals(2.0, dampingCoeff, 1e-6);

      dampingCoeff = GainCalculator.computeDampingForSecondOrderSystem(0, 1, 1);
      assertEquals(0.0, dampingCoeff, 1e-6);

      Random random = new Random();
      for (int i = 0; i < 1000; i++)
      {
         dampingCoeff = GainCalculator.computeDampingForSecondOrderSystem(random.nextDouble() * 1000 * -1, 1, 1);
         assertTrue(Double.isNaN(dampingCoeff));

         dampingCoeff = GainCalculator.computeDampingForSecondOrderSystem(1, random.nextDouble() * 1000 * -1, 1);
         assertTrue(Double.isNaN(dampingCoeff));
         
         dampingCoeff = GainCalculator.computeDampingForSecondOrderSystem(0, random.nextDouble() * 1000 * -1, 1);
         assertTrue(dampingCoeff == 0.0);

         dampingCoeff = GainCalculator.computeDampingForSecondOrderSystem(1, 1, random.nextDouble() * 1000 * -1);
         assertTrue(dampingCoeff < 0.0);
      }
   }

}
