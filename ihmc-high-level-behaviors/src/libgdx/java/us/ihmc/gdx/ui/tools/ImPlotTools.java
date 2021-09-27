package us.ihmc.gdx.ui.tools;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;

public final class ImPlotTools
{
   //Despite being identical code-wise, these need different methods because of casts from different primitive types
   public static Double[] convertArray(int[] array)
   {
      Double[] output = new Double[array.length];
      for (int i = 0; i < array.length; i++)
      {
         output[i] = (double) array[i];
      }
      return output;
   }

   public static Double[] convertArray(long[] array)
   {
      Double[] output = new Double[array.length];
      for (int i = 0; i < array.length; i++)
      {
         output[i] = (double) array[i];
      }
      return output;
   }

   public static Double[] convertArray(float[] array)
   {
      Double[] output = new Double[array.length];
      for (int i = 0; i < array.length; i++)
      {
         output[i] = (double) array[i];
      }
      return output;
   }

   public static Double[] convertArray(double[] array)
   {
      Double[] output = new Double[array.length];
      for (int i = 0; i < array.length; i++)
      {
         output[i] = array[i];
      }
      return output;
   }

   public static Double[] newNaNFilledBuffer(int bufferSize)
   {
      Double[] buffer = new Double[bufferSize];
      Arrays.fill(buffer, Double.NaN);
      return buffer;
   }

   public static <T extends Number> Integer[] createIndex(T[] array)
   {
      return createIndex(array, 0);
   }

   public static <T extends Number> Integer[] createIndex(T[] array, int start)
   {
      Integer[] output = new Integer[array.length];
      for (int i = 0; i < array.length; i++)
      {
         output[i] = i + start;
      }

      return output;
   }

   /** @deprecated THIS METHOD IS SUPER EXPENSIVE, DON'T USE */
   public static Double[] removeNullElements(Double[] array)
   {
      return removeNullElements(array, Double.class);
   }

   /** @deprecated THIS METHOD IS SUPER EXPENSIVE, DON'T USE */
   public static <T extends Number> T[] removeNullElements(T[] array, Class<T> arrayClass)
   {
      if (Arrays.stream(array).noneMatch(Objects::isNull))
         return array;

      ArrayList<T> output = new ArrayList<>();
      for (T t : array)
      {
         if (t != null)
            output.add(t);
      }

      T[] store = (T[]) Array.newInstance(arrayClass, output.size());
      return output.toArray(store);
   }
}
