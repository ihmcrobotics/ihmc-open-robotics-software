package us.ihmc.parameterTuner.guiElements.tuners;

import us.ihmc.parameterTuner.guiElements.GuiParameter;

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

   @Override
   public NumericSlider<Integer> createSlider()
   {
      return new IntegerSlider();
   }
}
