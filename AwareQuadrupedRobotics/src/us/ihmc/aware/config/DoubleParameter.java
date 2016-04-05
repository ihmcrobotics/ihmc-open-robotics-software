package us.ihmc.aware.config;

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
      notifyChangedListeners();
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
   protected String dumpValue()
   {
      return Double.toString(value);
   }
}

