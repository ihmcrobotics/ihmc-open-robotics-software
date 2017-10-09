package us.ihmc.simulationconstructionset;

public interface SimulationConstructor extends java.io.Serializable
{
   public Simulation constructSimulation(String[] structuralParameterNames, double[] structuralParameterValues);

   public void doActionAfterSimulationStateInitialized(Simulation simulation);
}
