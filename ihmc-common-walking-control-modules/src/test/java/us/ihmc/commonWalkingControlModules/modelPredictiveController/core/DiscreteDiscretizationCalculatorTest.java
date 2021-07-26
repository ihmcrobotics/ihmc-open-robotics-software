package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

public class DiscreteDiscretizationCalculatorTest extends DiscretizationCalculatorTest
{
   @Override
   public DiscretizationCalculator getCalculator()
   {
      return new DiscreteDiscretizationCalculator();
   }
}
