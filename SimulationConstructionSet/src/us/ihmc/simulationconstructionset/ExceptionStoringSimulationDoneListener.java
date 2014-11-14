package us.ihmc.simulationconstructionset;

public class ExceptionStoringSimulationDoneListener implements SimulationDoneListener
{
   private Throwable throwable;

   public void simulationDone()
   {
      // empty
   }

   public void simulationDoneWithException(Throwable throwable)
   {
      this.throwable = throwable;
   }

   public void reset()
   {
      throwable = null;
   }

   public Throwable getLastThrownException()
   {
      return throwable;
   }
}
