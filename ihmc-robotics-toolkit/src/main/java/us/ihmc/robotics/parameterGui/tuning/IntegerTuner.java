package us.ihmc.robotics.parameterGui.tuning;

import us.ihmc.robotics.parameterGui.GuiParameter;

public class IntegerTuner extends NumericTuner<Integer>
{
   public IntegerTuner(GuiParameter parameter)
   {
      super(parameter);
   }

   @Override
   public NumericSpinner<Integer> createASpinner()
   {
      return new IntegerSpinner();
   }

   @Override
   public Integer getSmallerNumber(Integer a, Integer b)
   {
      return Math.min(a, b);
   }

   @Override
   public Integer getLargerNumber(Integer a, Integer b)
   {
      return Math.max(a, b);
   }
}
