package us.ihmc.exampleSimulations.collisionExample;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;


public class CollisionExampleSimulation
{
   private SimulationConstructionSet sim;

   public CollisionExampleSimulation()
   {
      CollisionExampleRobot collisionExampleRobot = new CollisionExampleRobot();
      CollisionExampleController collisionExampleController = new CollisionExampleController(collisionExampleRobot);
      collisionExampleRobot.setController(collisionExampleController);

      sim = new SimulationConstructionSet(collisionExampleRobot);

      sim.setCameraFix(0.0, 0.0, 0.03);
      sim.setCameraPosition(-0.7, 1.0, 0.5);

      sim.setCameraTracking(false, true, true, false);
      sim.setCameraDolly(false, true, true, false);

      sim.setDT(0.004, 5);    // sim.setDT(0.0004,1);

      // Set up some graphs:

      sim.setupGraph("collision");
      sim.setupGraph("energy");

      sim.setupGraph(new String[] {"qd_ball1_wz", "qd_ball2_wz"});

      /*
       * sim.setupGraph(new String[] {"gc1_z"});
       * sim.setupGraph(new String[] {"gc2_z"});
       * sim.setupGraph(new String[] {"gc3_z"});
       * sim.setupGraph(new String[] {"gc4_z"});
       */

      /*
       * sim.setupGraph(new String[] {"gc5_z"});
       * sim.setupGraph(new String[] {"gc6_z"});
       * sim.setupGraph(new String[] {"gc7_z"});
       * sim.setupGraph(new String[] {"gc8_z"});
       */


      sim.setupEntryBox("qd_ball1_x");

      sim.setupEntryBox("q_ball1_x");
      sim.setupEntryBox("q_ball1_y");
      sim.setupEntryBox("q_ball1_z");

      // sim.setupEntryBox("ef_fy");

      sim.startOnAThread();
      sim.simulate(2.0);
   }

   public static void main(String[] args)
   {
      new CollisionExampleSimulation();
   }
}
