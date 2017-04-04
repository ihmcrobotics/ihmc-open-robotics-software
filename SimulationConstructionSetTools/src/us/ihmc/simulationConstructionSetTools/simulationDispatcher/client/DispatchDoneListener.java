package us.ihmc.simulationConstructionSetTools.simulationDispatcher.client;

public interface DispatchDoneListener
{
   public void dispatchDone(SimulationToDispatch dispatchSim, double[] finalState);
}
