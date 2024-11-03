package us.ihmc.tools.lists;

import java.util.List;
import java.util.Comparator;

public class ListSorter
{
   /**
    * Sort an array in place without allocating any memory.
    */
   public static <T> void sort(List<T> ts, Comparator<T> comparator)
   {
      boolean ordered = false;

      while (!ordered)
      {
         ordered = true;
         for (int i = 0; i < ts.size() - 1; i++)
         {
            T a = ts.get(i);
            T b = ts.get(i + 1);

            if(comparator.compare(a, b) > 0)
            {
               ordered = false;
               swap(ts, i, i + 1);
            }
         }
      }
   }

   public static <T> void swap(List<T> ts, int a, int b)
   {
      T tmp = ts.get(a);
      ts.set(a, ts.get(b));
      ts.set(b, tmp);
   }
}
