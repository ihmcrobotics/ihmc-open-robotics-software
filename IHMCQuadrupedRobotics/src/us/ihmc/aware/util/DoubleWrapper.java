package us.ihmc.aware.util;

public class DoubleWrapper
{
   private double value;

   public DoubleWrapper()
   {
      this(0.0);
   }

   public DoubleWrapper(double value)
   {
      this.value = value;
   }

   public double getValue()
   {
      return value;
   }

   public void setValue(double value)
   {
      this.value = value;
   }
}

