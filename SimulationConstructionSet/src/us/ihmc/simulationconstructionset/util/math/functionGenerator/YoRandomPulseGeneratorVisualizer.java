package us.ihmc.simulationconstructionset.util.math.functionGenerator;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public class YoRandomPulseGeneratorVisualizer implements RobotController
{
   private YoVariableRegistry registry = new YoVariableRegistry("YoFunGenViz");

   private YoRandomPulseGenerator yoRandomPulseGenerator;


   public YoRandomPulseGeneratorVisualizer(YoRandomPulseGenerator yoRandomPulseGenerator)
   {
      this.yoRandomPulseGenerator = yoRandomPulseGenerator;
   }

   public void initialize()
   {

   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return null;
   }

   public String getDescription()
   {
      return null;
   }



   public void doControl()
   {
     this.yoRandomPulseGenerator.getValue();


      try
      {
         Thread.sleep(1);
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }

   public static void main(String[] args)
   {
      Robot robot = new Robot("Robot");

      YoVariableRegistry registry = robot.getRobotsYoVariableRegistry();

      YoRandomPulseGenerator yoRandomPulseGenerator = new YoRandomPulseGenerator("PGen", robot.getYoTime(), registry);
      YoRandomPulseGeneratorVisualizer yoRandomPulseGeneratorVisualizer = new YoRandomPulseGeneratorVisualizer(yoRandomPulseGenerator);

      robot.setController(yoRandomPulseGeneratorVisualizer);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);

      scs.setDT(0.01, 1);

      Thread thread = new Thread(scs);

      thread.start();

//      try
//      {
//         Thread.sleep(1000);
//      }
//      catch (InterruptedException e)
//      {
//         e.printStackTrace();
//      }
//
      scs.hideViewport();
   }

}


