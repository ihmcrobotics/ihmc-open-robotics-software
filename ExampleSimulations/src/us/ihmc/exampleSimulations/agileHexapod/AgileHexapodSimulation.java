package us.ihmc.exampleSimulations.agileHexapod;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class AgileHexapodSimulation
{
   private SimulationConstructionSet sim;

   public AgileHexapodSimulation()
   {
      AgileHexapodRobot agileHexapod = new AgileHexapodRobot();
      // agileHexapod.createControllerBase(System.out, "AgileHexapod");

      sim = new SimulationConstructionSet(agileHexapod);

      sim.setCameraFix(-0.5, 0.0, 0.2);
      sim.setCameraPosition(0.5, 2.0, 0.3);

      sim.setCameraTracking(true, true, true, false);
      sim.setCameraDolly(false, true, true, false);

      sim.setDT(0.001, 5);

      // Set up some graphs:
      sim.setupGraph(new String[] { "q_z", "q_d_z" });
      sim.setupGraph(new String[] { "forward_angle" });

      sim.setupGraph(new String[] { "fZ", "fZa", "fZb" });
      sim.setupGraph(new String[] { "walk_state" });
      sim.setupGraph(new String[] { "vel_x", "vel_y" });
      sim.setupGraph(new String[] { "ground_height" });

      sim.setupEntryBox("vel_mag");
      sim.setupEntryBox("vel_ang");
      sim.setupEntryBox("qd_d_yaw");
      sim.setupEntryBox("lift_up");
      sim.setupEntryBox("hug_parallel");
      sim.setupEntryBox("leg_x_off");
      sim.setupEntryBox("leg_y_off");

      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new AgileHexapodSimulation();
   }
}