package us.ihmc.exampleSimulations.RobotArm;



import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.inputdevices.SliderBoardConfigurationManager;

public class RobotArmSimulation
{
   private SimulationConstructionSet sim;
   public RobotArmSimulation()
   {
      RobotArm robotArm = new RobotArm();

      RobotArmController controller = new RobotArmController(robotArm,"robotArm");
      robotArm.setController(controller);

      sim = new SimulationConstructionSet(robotArm);
      sim.setGroundVisible(false);
      sim.setDT(0.0000001, 10000);
      sim.setCameraFix(0.0, 0.0, 0.15);
      sim.setCameraPosition(0.0, -2.0, 0.0);

      SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(sim);
      sliderBoardConfigurationManager.setSlider(1, "axis1", controller.getYoVariableRegistry(), -0.25,3.15);
      sliderBoardConfigurationManager.setSlider(2, "axis2", controller.getYoVariableRegistry(), -1.026,2.574);
      sliderBoardConfigurationManager.setSlider(3, "axis3", controller.getYoVariableRegistry(), -1.649,1.151);
      sliderBoardConfigurationManager.setSlider(4, "axis4", controller.getYoVariableRegistry(), -0.578,2.722);
      sliderBoardConfigurationManager.setSlider(5, "axis5", controller.getYoVariableRegistry(), -0.954,1.1346);
      sliderBoardConfigurationManager.setSlider(6, "axis6", controller.getYoVariableRegistry(), -3.605,0.95);
      sliderBoardConfigurationManager.setSlider(7, "axis7", controller.getYoVariableRegistry(), -0.65,0.823);

      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new RobotArmSimulation();
   }
}

