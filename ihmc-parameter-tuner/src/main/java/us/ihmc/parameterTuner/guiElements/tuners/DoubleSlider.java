package us.ihmc.parameterTuner.guiElements.tuners;

public class DoubleSlider extends NumericSlider<Double>
{
   private static final int ticks = 100;

   @Override
   public double toDouble(Double number)
   {
      return number;
   }

   @Override
   public Double toNumber(double doubleValue)
   {
      return doubleValue;
   }

   @Override
   public int getPrecision()
   {
      double range = getMax() - getMin();
      double step = range / ticks;
      double precision = Math.log10(step);
      return (int) -Math.round(precision);
   }
}
