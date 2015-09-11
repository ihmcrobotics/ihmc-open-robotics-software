package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class DeadzoneYoVariable extends DoubleYoVariable implements ProcessingYoVariable
{
   private final DoubleYoVariable deadzoneSize;
   private final DoubleYoVariable inputVariable;

   public DeadzoneYoVariable(String name, DoubleYoVariable deadzoneSize, YoVariableRegistry registry)
   {
      super(name, registry);
      this.inputVariable = null;
      this.deadzoneSize = deadzoneSize;
   }

   public DeadzoneYoVariable(String name, DoubleYoVariable inputVariable, DoubleYoVariable deadzoneSize, YoVariableRegistry registry)
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
      update(inputVariable.getDoubleValue());
   }

   public void update(double valueToBeCorrected)
   {
      if (valueToBeCorrected >= deadzoneSize.getDoubleValue())
      {
         super.set(valueToBeCorrected - deadzoneSize.getDoubleValue());
      }
      else if (valueToBeCorrected <= -deadzoneSize.getDoubleValue())
      {
         super.set(valueToBeCorrected + deadzoneSize.getDoubleValue());
      }
      else
      {
         super.set(0.0);
      }
   }
   
   public static void main(String[] args)
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      DoubleYoVariable deadzoneSize = new DoubleYoVariable("deadzoneSize", registry);
      DoubleYoVariable input = new DoubleYoVariable("input", registry);
      deadzoneSize.set(2.0);
      DeadzoneYoVariable testDeadzone = new DeadzoneYoVariable("testDeadZone", input,deadzoneSize,registry);

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
