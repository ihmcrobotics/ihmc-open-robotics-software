package us.ihmc.math.linearAlgebra;

import us.ihmc.math.linearAlgebra.NullspaceCalculator;

public interface DampedNullspaceCalculator extends NullspaceCalculator
{
   public void setPseudoInverseAlpha(double alpha);
}
