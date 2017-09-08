package us.ihmc.simulationConstructionSetTools.simulationDispatcher.interfaces;

public interface RemoteSimulationDescription extends java.io.Serializable
{
   public abstract void createSimulation(String[] structuralParameterNames, double[] structuralParameterValues);

   public abstract void destroySimulation();

   public abstract void setSimulationState(Object state);
   
   public abstract void startSimulation();

   public abstract boolean isSimulationDone();

   public abstract Object getSimulationState();

   public abstract Object getSimulationData();
}
