package us.ihmc.simulationDispatcherExamples.springFlamingoDispatcher;

import us.ihmc.simulationconstructionset.Simulation;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class SpringFlamingoForDispatchSimulation
{
   private SimulationConstructionSet sim;
   public static final double DT = 0.0004;

   public SpringFlamingoForDispatchSimulation()
   {
      SpringFlamingoForDispatchSimulationConstructor springFlamingoSimulationConstructor = new SpringFlamingoForDispatchSimulationConstructor(4096, null);
      Simulation springFlamingoSim = springFlamingoSimulationConstructor.constructSimulation(null, null);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      sim = new SimulationConstructionSet(springFlamingoSim, parameters);


      sim.setCameraFix(0.0, 0.0, 0.6);
      sim.setCameraPosition(-9.0, 3.0, 0.8);

      sim.setCameraTracking(true, true, true, false);
      sim.setCameraDolly(false, true, true, false);

      sim.setDT(DT, 25);

      // Set up some graphs:

      sim.setupGraph("q_z");
      sim.setupGraph(new String[] {"left_state", "right_state", "t", "q_x"});


      sim.setupEntryBox(new String[]
      {
         "t_gain", "t_damp", "hip_d", "hip_hold", "hip_gain", "hip_damp", "swing_gain_knee", "swing_damp_knee"
      });

      sim.setFastSimulate(true);

      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      SpringFlamingoForDispatchSimulation sim = new SpringFlamingoForDispatchSimulation();
   }
}
