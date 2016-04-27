package us.ihmc.quadrupedRobotics.params;

public class DoubleParameter extends Parameter
{
   private double value;

   DoubleParameter(String path, double defaultValue)
   {
      super(path);
      this.value = defaultValue;
   }

   public double get()
   {
      return value;
   }

   public void set(double value)
   {
      this.value = value;
   }

   @Override
   public boolean tryLoadValue(String value)
   {
      try
      {
         this.value = Double.parseDouble(value);
         return true;
      } catch (NumberFormatException e)
      {
         return false;
      }
   }

   @Override
   String dumpValue()
   {
      return Double.toString(value);
   }
}

