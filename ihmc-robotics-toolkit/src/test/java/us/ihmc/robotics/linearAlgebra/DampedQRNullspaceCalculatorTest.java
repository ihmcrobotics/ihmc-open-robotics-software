package us.ihmc.robotics.linearAlgebra;

public class DampedQRNullspaceCalculatorTest extends DampedNullspaceCalculatorTest
{
   @Override
   public DampedNullspaceCalculator getDampedNullspaceProjectorCalculator()
   {
      return new DampedQRNullspaceCalculator(10, 0.0);
   }

   @Override
   public NullspaceCalculator getNullspaceProjectorCalculator()
   {
      return getDampedNullspaceProjectorCalculator();
   }
}
