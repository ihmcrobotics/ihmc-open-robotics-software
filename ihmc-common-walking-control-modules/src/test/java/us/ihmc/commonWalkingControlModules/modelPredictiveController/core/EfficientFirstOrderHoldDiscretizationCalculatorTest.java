package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

public class EfficientFirstOrderHoldDiscretizationCalculatorTest extends DiscretizationCalculatorTest
{
   @Override
   public DiscretizationCalculator getCalculator()
   {
      return new EfficientFirstOrderHoldDiscretizationCalculator();
   }


   @Disabled
   @Test
   public void testEquivalent()
   {
      super.testEquivalent();
   }
}
