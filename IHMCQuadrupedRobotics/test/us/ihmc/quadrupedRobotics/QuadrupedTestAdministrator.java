package us.ihmc.quadrupedRobotics;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;

public class QuadrupedTestAdministrator
{
   private final SimulationConstructionSet scs;
   private final BlockingSimulationRunner blockingSimulationRunner;
   
   public QuadrupedTestAdministrator(SimulationConstructionSet scs)
   {
      this.scs = scs;
      this.blockingSimulationRunner = new BlockingSimulationRunner(scs, 600.0, true);
      
      scs.startOnAThread();
   }
   
   public void simulate(double time)
   {
      try
      {
         blockingSimulationRunner.simulateAndBlock(time);
      }
      catch (SimulationExceededMaximumTimeException | ControllerFailureException e)
      {
         e.printStackTrace();
      }
   }
   
   // TODO
   public void simulateUntilEvent()
   {
      // pass in some event object
   }
}
