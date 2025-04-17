package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class StabilityCalculatorComparer
{
   private final YoDouble error;
   private final StabilityMarginRegionCalculator baselineCalculator;
   private final StabilityMarginRegionCalculator calculatorToTest;

   public StabilityCalculatorComparer(String namePrefix, StabilityMarginRegionCalculator baselineCalculator, StabilityMarginRegionCalculator calculatorToTest, YoRegistry registry)
   {
      error = new YoDouble(namePrefix + "MarginError", registry);
      this.baselineCalculator = baselineCalculator;
      this.calculatorToTest = calculatorToTest;
   }

   public void update()
   {
      this.error.set(Math.abs(baselineCalculator.getStabilityMargin() - calculatorToTest.getStabilityMargin()));
   }
}
