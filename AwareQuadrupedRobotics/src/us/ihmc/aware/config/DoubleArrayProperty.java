package us.ihmc.aware.config;

import com.google.common.primitives.Doubles;

public class DoubleArrayProperty extends Property
{
   private double[] value;

   protected DoubleArrayProperty(String path, double... defaultValue)
   {
      super(path);
      this.value = defaultValue;
   }

   public double[] get()
   {
      return value;
   }

   public void set(double[] value)
   {
      this.value = value;
   }

   public void set(int idx, double value)
   {
      this.value[idx] = value;
   }

   @Override
   public String dump()
   {
      return getPath() + "=" + Doubles.join(",", value);
   }
}
