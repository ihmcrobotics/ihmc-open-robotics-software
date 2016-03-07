package us.ihmc.simulationconstructionset;

public interface SimulationDoneListener
{
   public abstract void simulationDone();
   public abstract void simulationDoneWithException(Throwable throwable);
}
