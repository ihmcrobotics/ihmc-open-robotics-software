package us.ihmc.robotics.linearAlgebra;

import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class DampedLeastSquaresNullspaceCalculatorTest extends NullspaceProjectorCalculatorTest
{
   @Override
   public NullspaceProjectorCalculator getNullspaceProjectorCalculator()
   {
      return new DampedLeastSquaresNullspaceCalculator(10, 0.0, new YoVariableRegistry(getClass().getSimpleName()));
   }
}
