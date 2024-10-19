package us.ihmc.math.linearAlgebra;

/**
 * @author twan
 * Date: 4/11/13
 */
public class QRNullspaceCalculatorTest extends NullspaceCalculatorTest
{
   @Override
   public NullspaceCalculator getNullspaceProjectorCalculator()
   {
      return new QRNullspaceCalculator(10);
   }

}
