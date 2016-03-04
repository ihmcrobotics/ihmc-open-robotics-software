package us.ihmc.simulationconstructionset.simulatedSensors;

public class UnexpectedConcurrancyException extends Exception
{
   private static final long serialVersionUID = -2399934723168629373L;

   public UnexpectedConcurrancyException(String message)
   {
      super(message);
   }
}
