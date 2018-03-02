package us.ihmc.robotics.dataStructures.parameter;

import us.ihmc.yoVariables.providers.DoubleProvider;

public class DoubleParameter extends Parameter implements DoubleProvider
{
   private double value;

   private DoubleParameter() // no-arg for serialization
   {
      super(null);
      this.value =  0.0;
   }

   public DoubleParameter(String path, double defaultValue)
   {
      super(path);
      this.value = defaultValue;
   }

   public double get()
   {
      return value;
   }

   public double getValue()
   {
      return get();
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

