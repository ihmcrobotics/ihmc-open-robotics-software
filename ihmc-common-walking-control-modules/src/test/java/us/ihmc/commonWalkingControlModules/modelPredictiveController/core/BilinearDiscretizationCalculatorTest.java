package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

public class BilinearDiscretizationCalculatorTest extends DiscretizationCalculatorTest
{
   @Override
   public DiscretizationCalculator getCalculator()
   {
      return new BilinearDiscretizationCalculator();
   }
}
