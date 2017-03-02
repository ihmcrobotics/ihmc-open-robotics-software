package us.ihmc.robotics.dataStructures.parameter;

import org.apache.commons.lang3.StringUtils;

import com.google.common.primitives.Ints;

public class IntegerArrayParameter extends Parameter
{
   private int[] value;

   private IntegerArrayParameter() // no-arg for serialization
   {
      super(null);
      this.value =  new int[0];
   }

   IntegerArrayParameter(String path, int... defaultValue)
   {
      super(path);
      this.value = defaultValue;
   }

   public int[] get()
   {
      return value;
   }

   public int get(int idx)
   {
      return get()[idx];
   }

   public void set(int[] value)
   {
      this.value = value;
      super.notifyChangeListeners();
   }

   public void set(int idx, int value)
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
         String[] ints = csv.split(",");

         if (ints.length != this.value.length)
         {
            System.err.println("Invalid int array parameter length: expected " + this.value.length + " got " + ints.length);
            return false;
         }

         for (int i = 0; i < ints.length; i++)
         {
            set(i, Integer.parseInt(ints[i]));
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
      return Ints.join(",", value);
   }
}
