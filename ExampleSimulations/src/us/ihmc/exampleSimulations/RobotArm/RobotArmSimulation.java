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

      // Assign sliders to axes
      SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(sim);
      sliderBoardConfigurationManager.setSlider(1, "axis1", controller.getYoVariableRegistry(), -Math.toRadians(90), Math.toRadians(90));
      sliderBoardConfigurationManager.setSlider(2, "axis2", controller.getYoVariableRegistry(), -Math.toRadians(90), Math.toRadians(90));
      sliderBoardConfigurationManager.setSlider(3, "axis3", controller.getYoVariableRegistry(), -Math.toRadians(90), Math.toRadians(90));
      sliderBoardConfigurationManager.setSlider(4, "axis4", controller.getYoVariableRegistry(), -Math.toRadians(90), Math.toRadians(90));
      sliderBoardConfigurationManager.setSlider(5, "axis5", controller.getYoVariableRegistry(), -Math.toRadians(90), Math.toRadians(90));
      sliderBoardConfigurationManager.setSlider(6, "axis6", controller.getYoVariableRegistry(), -Math.toRadians(90), Math.toRadians(90));
      sliderBoardConfigurationManager.setSlider(7, "axis7Left", controller.getYoVariableRegistry(), -Math.toRadians(45), Math.toRadians(45));
//      sliderBoardConfigurationManager.setSlider(7, "axis7Right", controller.getYoVariableRegistry(), -Math.toRadians(90), Math.toRadians(90));

      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new RobotArmSimulation();
   }
}

