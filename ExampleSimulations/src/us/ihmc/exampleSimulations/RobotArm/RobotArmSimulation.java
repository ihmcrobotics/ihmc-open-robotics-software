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
     // sim = new SimulationConstructionSet();
      sim.setGroundVisible(false);
      sim.setDT(0.0000001, 10000);
      sim.setCameraPosition(0, -40.0, 2.0);

      SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(sim);
      sliderBoardConfigurationManager.setSlider(1, "Turning1", controller.getYoVariableRegistry(), -0.25,3.15);
      sliderBoardConfigurationManager.setSlider(2, "longStick1", controller.getYoVariableRegistry(), -1.026,2.574);
      sliderBoardConfigurationManager.setSlider(3, "longStick2", controller.getYoVariableRegistry(), -1.649,1.151);
      sliderBoardConfigurationManager.setSlider(4, "Turning2", controller.getYoVariableRegistry(), -0.578,2.722);

      sliderBoardConfigurationManager.setSlider(5, "longStick3", controller.getYoVariableRegistry(), -0.954,1.1346);
      sliderBoardConfigurationManager.setSlider(6, "Turning3", controller.getYoVariableRegistry(), -3.605,0.95);
      sliderBoardConfigurationManager.setSlider(7, "longStick4", controller.getYoVariableRegistry(), -0.65,0.823);



      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new RobotArmSimulation();
   }
}

