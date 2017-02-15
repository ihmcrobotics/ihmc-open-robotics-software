package us.ihmc.simulationconstructionset;

public class ExceptionStoringSimulationDoneListener implements SimulationDoneListener
{
   private Throwable throwable;

   @Override
   public void simulationDone()
   {
      // empty
   }

   @Override
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
