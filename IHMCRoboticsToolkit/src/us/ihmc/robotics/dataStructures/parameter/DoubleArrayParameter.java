package us.ihmc.robotics.dataStructures.parameter;

import org.apache.commons.lang3.StringUtils;

import com.google.common.primitives.Doubles;

public class DoubleArrayParameter extends Parameter
{
   private double[] value;

   private DoubleArrayParameter() // no-arg for serialization
   {
      super(null);
      this.value =  new double[0];
   }

   DoubleArrayParameter(String path, double... defaultValue)
   {
      super(path);
      this.value = defaultValue;
   }

   public double[] get()
   {
      return value;
   }

   public double get(int idx)
   {
      return get()[idx];
   }

   public void set(double[] value)
   {
      this.value = value;
      super.notifyChangeListeners();
   }

   public void set(int idx, double value)
   {
      this.value[idx] = value;
      super.notifyChangeListeners();
   }

   @Override
   public boolean tryLoadValue(String value)
   {
      try
      {
         String csv = StringUtils.strip(value, "{}");
         String[] doubles = csv.split(",");

         if (doubles.length != this.value.length)
         {
            System.err.println("Invalid double array parameter length: expected " + this.value.length + " got " + doubles.length);
            return false;
         }

         for (int i = 0; i < doubles.length; i++)
         {
            set(i, Double.parseDouble(doubles[i]));
         }

         return true;
      } catch (NumberFormatException e)
      {
         return false;
      }
   }

   @Override
   String dumpValue()
   {
      return Doubles.join(",", value);
   }
}
