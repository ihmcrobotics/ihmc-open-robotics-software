package us.ihmc.robotics.math.functionGenerator;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;

public class YoRandomPulseGeneratorVisualizer implements RobotController
{
   private YoVariableRegistry registry = new YoVariableRegistry("YoFunGenViz");

   private YoRandomPulseGenerator yoRandomPulseGenerator;


   public YoRandomPulseGeneratorVisualizer(YoRandomPulseGenerator yoRandomPulseGenerator)
   {
      this.yoRandomPulseGenerator = yoRandomPulseGenerator;
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return null;
   }

   @Override
   public String getDescription()
   {
      return null;
   }



   @Override
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
}


