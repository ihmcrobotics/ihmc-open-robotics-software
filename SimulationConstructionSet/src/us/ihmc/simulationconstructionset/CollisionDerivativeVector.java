package us.ihmc.simulationconstructionset;

class CollisionDerivativeException extends Exception implements java.io.Serializable
{
   /**
    *
    */
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
   public void derivs(double x, double[] y, double[] dydx) throws CollisionDerivativeException;

   public boolean isStuck(double[] y);

}
