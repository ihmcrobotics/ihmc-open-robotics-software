package us.ihmc.robotics.parameterGui.tuning;

public class IntegerSlider extends NumericSlider<Integer>
{
   @Override
   public double doDouble(Integer number)
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
