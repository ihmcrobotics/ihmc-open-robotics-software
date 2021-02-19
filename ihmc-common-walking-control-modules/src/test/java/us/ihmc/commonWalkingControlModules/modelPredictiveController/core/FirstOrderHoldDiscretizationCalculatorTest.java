package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

public class FirstOrderHoldDiscretizationCalculatorTest extends DiscretizationCalculatorTest
{
   @Override
   public DiscretizationCalculator getCalculator()
   {
      return new FirstOrderHoldDiscretizationCalculator();
   }
}
