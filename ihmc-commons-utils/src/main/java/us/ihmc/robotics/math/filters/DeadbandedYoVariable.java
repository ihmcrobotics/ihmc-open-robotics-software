package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.math.DeadbandTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DeadbandedYoVariable extends YoDouble implements ProcessingYoVariable
{
   private final DoubleProvider deadzoneSize;
   private final DoubleProvider inputVariable;

   public DeadbandedYoVariable(String name, DoubleProvider deadzoneSize, YoRegistry registry)
   {
      super(name, registry);
      this.inputVariable = null;
      this.deadzoneSize = deadzoneSize;
   }

   public DeadbandedYoVariable(String name, DoubleProvider inputVariable, DoubleProvider deadzoneSize, YoRegistry registry)
   {
      super(name, registry);
      this.inputVariable = inputVariable;
      this.deadzoneSize = deadzoneSize;
   }

   public void update()
   {
      if (inputVariable == null)
         throw new NullPointerException("DeadzoneYoVariable must be constructed with a non null "
               + "input variable to call update(), otherwise use update(double)");
      update(inputVariable.getValue());
   }

   public void update(double valueToBeCorrected)
   {
      super.set(DeadbandTools.applyDeadband(deadzoneSize.getValue(), valueToBeCorrected));
   }
   
   public static void main(String[] args)
   {
      YoRegistry registry = new YoRegistry("test");
      YoDouble deadzoneSize = new YoDouble("deadzoneSize", registry);
      YoDouble input = new YoDouble("input", registry);
      deadzoneSize.set(2.0);
      DeadbandedYoVariable testDeadzone = new DeadbandedYoVariable("testDeadZone", input, deadzoneSize, registry);

      for(int i = -50; i < 51; i++)
      {
         input.set((double)i);
         testDeadzone.update();
         System.out.println("//////////////////////////");
         System.out.println("uncorrected = " + (double)i);
         System.out.println("corrected = " + testDeadzone.getDoubleValue());
      }
      
      System.out.println("done");
   }
   
}
