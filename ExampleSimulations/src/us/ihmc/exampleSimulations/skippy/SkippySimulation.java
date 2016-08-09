package us.ihmc.exampleSimulations.skippy;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class SkippySimulation
{
   public static final double DT = 0.0001;
   public static final double TIME = 2.0;
   private static SimulationConstructionSet sim;

   public SkippySimulation()
   {
      SkippyRobot skippy = new SkippyRobot();
      skippy.setController(new SkippyController(skippy,"skippyController"));
//      skippy.setController(new ExternalControlServer(skippy, "externalControlServer"));

      sim = new SimulationConstructionSet(skippy);
      sim.setGroundVisible(true);
      sim.setDT(DT, 20);
      sim.setSimulateDuration(TIME);
      sim.setCameraPosition(-80, 0.0, -2.0);

      Thread myThread = new Thread(sim);
      myThread.start();
   }

   /*
    * When your simulation is run, first the main method will be called.
    * In creating a Skippy Simulation, a SkippyRobot is first created, and then a Simulation Construction Set object is created with that robot.
    * A Thread is then created using the SimulationConstructionSet object.
    * Finally the Thread is started, thereby starting your simulation.
    */

   public static void main(String[] args)
   {
      new SkippySimulation();
   }
}