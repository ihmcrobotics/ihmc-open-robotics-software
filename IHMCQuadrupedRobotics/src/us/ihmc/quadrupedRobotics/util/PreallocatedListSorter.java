package us.ihmc.quadrupedRobotics.util;

import java.util.Comparator;

public class PreallocatedListSorter
{
   /**
    * Sort an array in place without allocating any memory.
    */
   public static <T> void sort(PreallocatedList<T> ts, Comparator<T> comparator)
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
               ts.swap(i, i + 1);
            }
         }
      }
   }
}
