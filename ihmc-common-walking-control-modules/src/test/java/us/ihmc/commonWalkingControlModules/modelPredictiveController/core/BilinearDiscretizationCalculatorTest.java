package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

public class BilinearDiscretizationCalculatorTest extends DiscretizationCalculatorTest
{
   @Override
   public DiscretizationCalculator getCalculator()
   {
      return new BilinearDiscretizationCalculator();
   }

   @Override
   public double getEpsilon()
   {
      return 5e-1;
   }

   @Disabled
   @Test
   public void testEquivalent()
   {
      super.testEquivalent();
   }
}
