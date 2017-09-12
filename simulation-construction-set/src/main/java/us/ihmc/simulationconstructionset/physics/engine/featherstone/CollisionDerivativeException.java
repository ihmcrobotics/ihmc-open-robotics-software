package us.ihmc.simulationconstructionset.physics.engine.featherstone;

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
