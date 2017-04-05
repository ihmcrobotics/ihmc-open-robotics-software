package us.ihmc.simulationConstructionSetTools.simulationTesting;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class AllYoVariablesSimulationComparer extends YoVariableListComparer implements SimulationComparer
{
   public AllYoVariablesSimulationComparer(double epsilon)
   {
      super(epsilon);
   }

   @Override
   public boolean compare(SimulationConstructionSet scs0, SimulationConstructionSet scs1)
   {
      return compare(scs0.getAllVariables(), scs1.getAllVariables());
   }
}
