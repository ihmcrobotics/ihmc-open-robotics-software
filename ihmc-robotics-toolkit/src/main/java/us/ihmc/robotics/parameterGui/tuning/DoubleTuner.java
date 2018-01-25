package us.ihmc.robotics.parameterGui.tuning;

import us.ihmc.robotics.parameterGui.GuiParameter;

public class DoubleTuner extends NumericTuner<Double>
{
   public DoubleTuner(GuiParameter parameter)
   {
      super(parameter);
   }

   @Override
   public NumericSpinner<Double> createSpinner()
   {
      return new DoubleSpinner();
   }
}
