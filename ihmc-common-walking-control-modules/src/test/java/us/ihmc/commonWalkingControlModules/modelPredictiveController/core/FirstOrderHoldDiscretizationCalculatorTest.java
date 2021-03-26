package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

public class FirstOrderHoldDiscretizationCalculatorTest extends DiscretizationCalculatorTest
{
   @Override
   public DiscretizationCalculator getCalculator()
   {
      return new FirstOrderHoldDiscretizationCalculator();
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
