package us.ihmc.exampleSimulations.buildingPendulum;

import us.ihmc.exampleSimulations.DoublePendulum.DoublePendulumRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class BuildingPendulumSimulation
{
   private SimulationConstructionSet sim;
   public BuildingPendulumSimulation()
   {
      BuildingPendulumRobot buildingPendulumRobot = new BuildingPendulumRobot();

      buildingPendulumRobot.setController(new BuildingPendulumController(buildingPendulumRobot));

      sim = new SimulationConstructionSet(buildingPendulumRobot);
      sim.setGroundVisible(false);
      sim.setCameraPosition(0, -40.0, 2.0);

      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new BuildingPendulumSimulation();
   }
}
