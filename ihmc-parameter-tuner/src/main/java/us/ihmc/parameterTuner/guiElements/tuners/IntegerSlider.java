package us.ihmc.parameterTuner.guiElements.tuners;

public class IntegerSlider extends NumericSlider<Integer>
{
   @Override
   public double toDouble(Integer number)
   {
      return number;
   }

   @Override
   public Integer toNumber(double doubleValue)
   {
      return (int) Math.round(doubleValue);
   }

   @Override
   public int getPrecision()
   {
      return 0;
   }
}
