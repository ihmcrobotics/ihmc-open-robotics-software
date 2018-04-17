package us.ihmc.parameterTuner.guiElements.tuners;

import us.ihmc.parameterTuner.guiElements.GuiParameter;

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

   @Override
   public NumericSlider<Double> createSlider()
   {
      return new DoubleSlider();
   }
}
