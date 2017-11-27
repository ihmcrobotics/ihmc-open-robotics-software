package us.ihmc.robotics.linearAlgebra;

public interface DampedNullspaceCalculator extends NullspaceProjectorCalculator
{
   public void setPseudoInverseAlpha(double alpha);
}
