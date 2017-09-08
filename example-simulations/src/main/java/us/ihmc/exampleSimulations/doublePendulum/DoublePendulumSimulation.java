package us.ihmc.exampleSimulations.doublePendulum;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class DoublePendulumSimulation
{
   private SimulationConstructionSet sim;
   public DoublePendulumSimulation()
   {
      DoublePendulumRobot doublePendulum = new DoublePendulumRobot();
      doublePendulum.setController(new DoublePendulumController(doublePendulum,"doublePendulumController"));
      sim = new SimulationConstructionSet(doublePendulum);
      sim.setGroundVisible(false);
      sim.setCameraPosition(0, -40.0, 2.0);

      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new DoublePendulumSimulation();
   }
}