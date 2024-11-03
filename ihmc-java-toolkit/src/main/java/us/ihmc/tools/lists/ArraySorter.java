package us.ihmc.tools.lists;

import java.util.Comparator;

public class ArraySorter
{
   /**
    * Sort an array in place without allocating any memory.
    */
   public static <T> void sort(T[] ts, Comparator<T> comparator)
   {
      boolean ordered = false;

      while (!ordered)
      {
         ordered = true;
         for (int i = 0; i < ts.length - 1; i++)
         {
            T a = ts[i];
            T b = ts[i + 1];

            if(comparator.compare(a, b) > 0)
            {
               ordered = false;
               swap(ts, i, i + 1);
            }
         }
      }
   }

   public static <T> void swap(T[] ts, int a, int b)
   {
      T tmp = ts[a];
      ts[a] = ts[b];
      ts[b] = tmp;
   }
}













