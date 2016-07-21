package us.ihmc.exampleSimulations.skippy;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class SkippySimulation
{
   private SimulationConstructionSet sim;

   public SkippySimulation()
   {
      SkippyRobot skippyRobot = new SkippyRobot();
      skippyRobot.setController(new SkippyController(skippyRobot,"skippyController"));
      //      skippy.setController(new ExternalControlServer(skippy, "externalControlServer"));

      sim = new SimulationConstructionSet(skippyRobot);
      sim.setGroundVisible(false);
      sim.setCameraPosition(0, -40.0, 2.0);

      sim.setSimulateDuration(3.0);
      Thread myThread = new Thread(sim);
      myThread.start();
   }

   /*
    * When your simulation is run, first the main method will be called. It creates a new SkippySimulation. In creating a SkippySimulation, a SkippyRobot is first created, and then a SimulationConstructionSet object is created with that robot. A Thread is then created using the SimulationConstructionSet object. Finally the Thread is started, thereby starting your simulation. This simple template can be used for creating any simulation using the Simulation Construction Set.
    */

   public static void main(String[] args)
   {
      new SkippySimulation();
   }
}