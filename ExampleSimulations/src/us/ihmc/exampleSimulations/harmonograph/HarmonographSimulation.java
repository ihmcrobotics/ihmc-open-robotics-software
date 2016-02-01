package us.ihmc.exampleSimulations.harmonograph;


import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class HarmonographSimulation
{

   private static final double simulateDT = 0.0002;
   private static final int recordFrequency = 100;


   public HarmonographSimulation()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      HarmonographRobot robot = new HarmonographRobot(yoGraphicsListRegistry);
      
      SimulationConstructionSet simulationConstructionSet = new SimulationConstructionSet(robot);
      simulationConstructionSet.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      
      simulationConstructionSet.setDT(simulateDT, recordFrequency);
      
      HarmonographPaperJPanel harmonographPaperJPanel = robot.getHarmonographPaperJPanel();
      simulationConstructionSet.addExtraJpanel(harmonographPaperJPanel, "Harmonograph Paper");
      
      Thread thread = new Thread(simulationConstructionSet);
      thread.start();
   }
   
   
   public static void main(String[] args)
   {
      new HarmonographSimulation();
   }

}
