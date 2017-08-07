package us.ihmc.simulationConstructionSetTools.simulationTesting;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public interface SimulationComparisonScript
{
   public abstract void doInitialAction(SimulationConstructionSet scs0, SimulationConstructionSet scs1);
   public abstract void doFinalAction(SimulationConstructionSet scs0, SimulationConstructionSet scs1);
}
