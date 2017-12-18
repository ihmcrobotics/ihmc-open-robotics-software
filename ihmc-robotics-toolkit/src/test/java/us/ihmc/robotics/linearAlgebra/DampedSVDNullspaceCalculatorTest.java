package us.ihmc.robotics.linearAlgebra;

public class DampedSVDNullspaceCalculatorTest extends DampedNullspaceCalculatorTest
{
   @Override
   public DampedNullspaceCalculator getDampedNullspaceProjectorCalculator()
   {
      return new DampedSVDNullspaceCalculator(10, 0.0);
   }

   @Override
   public NullspaceCalculator getNullspaceProjectorCalculator()
   {
      return getDampedNullspaceProjectorCalculator();
   }
}
