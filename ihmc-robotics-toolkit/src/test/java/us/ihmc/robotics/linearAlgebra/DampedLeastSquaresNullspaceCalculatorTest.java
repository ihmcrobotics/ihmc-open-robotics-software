package us.ihmc.robotics.linearAlgebra;

public class DampedLeastSquaresNullspaceCalculatorTest extends DampedNullspaceCalculatorTest
{
   @Override
   public DampedNullspaceCalculator getDampedNullspaceProjectorCalculator()
   {
      return new DampedLeastSquaresNullspaceCalculator(10, 0.0);
   }

   @Override
   public NullspaceCalculator getNullspaceProjectorCalculator()
   {
      return getDampedNullspaceProjectorCalculator();
   }
}
