package us.ihmc.exampleSimulations.recursivePendulum;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class RecursivePendulumSimulation
{
   private SimulationConstructionSet sim;

   public RecursivePendulumSimulation()
   {
      RecursivePendulumRobot recursivePendulum = new RecursivePendulumRobot();
      sim = new SimulationConstructionSet(recursivePendulum);
      sim.setDT(0.002, 10);

      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new RecursivePendulumSimulation();
   }
}
