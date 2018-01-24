package us.ihmc.robotics.parameterGui.tuning;

import us.ihmc.robotics.parameterGui.GuiParameter;

public class DoubleTuner extends NumericTuner<Double>
{
   public DoubleTuner(GuiParameter parameter)
   {
      super(parameter);
   }

   @Override
   public NumericSpinner<Double> createASpinner()
   {
      return new DoubleSpinner();
   }

   @Override
   public boolean areBoundsConsistent(Double value, Double min, Double max)
   {
      return value >= min && value <= max;
   }

   @Override
   public Double getSmallerNumber(Double a, Double b)
   {
      return Math.min(a, b);
   }

   @Override
   public Double getLargerNumber(Double a, Double b)
   {
      return Math.max(a, b);
   }
}
