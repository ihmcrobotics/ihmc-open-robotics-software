package us.ihmc.robotics.math.functionGenerator;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoRandomPulseGeneratorVisualizer implements RobotController
{
   private YoRegistry registry = new YoRegistry("YoFunGenViz");

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
   public YoRegistry getYoRegistry()
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


