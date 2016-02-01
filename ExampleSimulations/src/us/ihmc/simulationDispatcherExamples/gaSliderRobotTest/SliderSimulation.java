package us.ihmc.simulationDispatcherExamples.gaSliderRobotTest;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;


public class SliderSimulation
{
   private SimulationConstructionSet sim;

   public SliderSimulation()
   {
      SliderRobot sliderRobot = new SliderRobot(null, null);
      SliderController controller = new SliderController(sliderRobot);
      sliderRobot.setController(controller);

      sim = new SimulationConstructionSet(sliderRobot);
      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      SliderSimulation sim = new SliderSimulation();
   }
}
