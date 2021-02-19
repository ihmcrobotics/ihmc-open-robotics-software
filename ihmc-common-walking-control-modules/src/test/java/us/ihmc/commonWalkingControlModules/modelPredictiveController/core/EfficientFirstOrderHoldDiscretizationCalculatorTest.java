package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

public class EfficientFirstOrderHoldDiscretizationCalculatorTest extends DiscretizationCalculatorTest
{
   @Override
   public DiscretizationCalculator getCalculator()
   {
      return new EfficientFirstOrderHoldDiscretizationCalculator();
   }
}
