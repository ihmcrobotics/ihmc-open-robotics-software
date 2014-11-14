package us.ihmc.simulationconstructionset.simulatedSensors;

public class NotReadyException extends Exception
{
   private static final long serialVersionUID = 8684291629565409542L;

   public NotReadyException (String message)
   {
      super(message);
   }
}
