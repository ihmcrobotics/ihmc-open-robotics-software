package us.ihmc.simulationconstructionset.physics.engine.featherstone;

public interface CollisionDerivativeVector extends java.io.Serializable
{
   public abstract void derivs(double x, double[] y, double[] dydx) throws CollisionDerivativeException;

   public abstract boolean isStuck(double[] y);
}
