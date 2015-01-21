package us.ihmc.simulationconstructionset.simulationDispatcher.client;

public interface DispatchDoneListener
{
   public void dispatchDone(SimulationToDispatch dispatchSim, double[] finalState);
}
