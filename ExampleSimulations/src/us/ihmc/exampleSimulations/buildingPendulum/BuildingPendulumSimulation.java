package us.ihmc.exampleSimulations.buildingPendulum;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationConstructionSetTools.util.inputdevices.SliderBoardConfigurationManager;

public class BuildingPendulumSimulation
{
   private SimulationConstructionSet sim;
   public BuildingPendulumSimulation()
   {
      BuildingPendulumRobot buildingPendulumRobot = new BuildingPendulumRobot();
      BuildingPendulumController controller = new BuildingPendulumController(buildingPendulumRobot);
      buildingPendulumRobot.setController(controller);

      sim = new SimulationConstructionSet(buildingPendulumRobot);
      sim.setGroundVisible(false);
      sim.setDT(0.00001, 400);
      sim.setCameraPosition(0, -40.0, 2.0);

      SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(sim);
      sliderBoardConfigurationManager.setSlider(1, BuildingPendulumController.variableName, controller.getYoVariableRegistry(), -1.0, 1.0);

      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new BuildingPendulumSimulation();
   }
}
