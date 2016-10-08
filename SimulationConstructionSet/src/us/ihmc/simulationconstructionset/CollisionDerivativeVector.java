package us.ihmc.simulationconstructionset;

class CollisionDerivativeException extends Exception implements java.io.Serializable
{
   private static final long serialVersionUID = 2874294625173771625L;

   public CollisionDerivativeException()
   {
   }

   public CollisionDerivativeException(String msg)
   {
      super(msg);
   }
}


public interface CollisionDerivativeVector extends java.io.Serializable
{
   public abstract void derivs(double x, double[] y, double[] dydx) throws CollisionDerivativeException;
   public abstract boolean isStuck(double[] y);
}
