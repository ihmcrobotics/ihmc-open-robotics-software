package us.ihmc.exampleSimulations.doublePendulum;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class DoublePendulumSimulation
{
   private SimulationConstructionSet sim;

   public DoublePendulumSimulation()
   {
      DoublePendulumRobot doublePendulum = new DoublePendulumRobot();
      doublePendulum.setController(new DoublePendulumController(doublePendulum,"doublePendulumController"));
//      doublePendulum.setController(new ExternalControlServer(doublePendulum, "externalControlServer"));

      sim = new SimulationConstructionSet(doublePendulum);
      sim.setGroundVisible(false);
      sim.setCameraPosition(0, -40.0, 2.0);
      
      sim.setSimulateDuration(3.0);
      Thread myThread = new Thread(sim);
      myThread.start();
   }

   /*
    * When your simulation is run, first the main method will be called. It creates a new
    * DoublePendulumSimulation. In creating a DoublePendulumSimulation, a
    * DoublePendulumRobot is first created, and then a SimulationConstructionSet object is
    * created with that robot. A Thread is then created using the SimulationConstructionSet
    * object. Finally the Thread is started, thereby starting your simulation. This simple template
    * can be used for creating any simulation using the Simulation Construction Set.
    */
   
   public static void main(String[] args)
   {
      new DoublePendulumSimulation();
   }
}
