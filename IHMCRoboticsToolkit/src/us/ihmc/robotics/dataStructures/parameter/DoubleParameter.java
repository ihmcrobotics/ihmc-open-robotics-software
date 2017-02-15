package us.ihmc.robotics.dataStructures.parameter;

public class DoubleParameter extends Parameter
{
   private double value;

   private DoubleParameter() // no-arg for serialization
   {
      super(null);
      this.value =  0.0;
   }

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
      super.notifyChangeListeners();
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

