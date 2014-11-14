package us.ihmc.simulationconstructionset.util.simulationTesting;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public interface SimulationComparer
{
   public abstract boolean compare(SimulationConstructionSet scs0, SimulationConstructionSet scs1);
}
