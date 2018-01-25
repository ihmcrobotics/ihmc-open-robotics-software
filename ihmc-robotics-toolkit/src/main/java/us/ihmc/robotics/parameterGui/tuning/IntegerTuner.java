package us.ihmc.robotics.parameterGui.tuning;

import us.ihmc.robotics.parameterGui.GuiParameter;

public class IntegerTuner extends NumericTuner<Integer>
{
   public IntegerTuner(GuiParameter parameter)
   {
      super(parameter);
   }

   @Override
   public NumericSpinner<Integer> createSpinner()
   {
      return new IntegerSpinner();
   }
}
