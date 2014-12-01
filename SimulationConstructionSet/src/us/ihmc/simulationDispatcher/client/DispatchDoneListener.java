package us.ihmc.simulationDispatcher.client;

public interface DispatchDoneListener
{
   public void dispatchDone(SimulationToDispatch dispatchSim, double[] finalState);
}
