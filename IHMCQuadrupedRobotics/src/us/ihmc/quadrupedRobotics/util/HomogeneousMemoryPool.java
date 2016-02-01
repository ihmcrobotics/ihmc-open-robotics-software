package us.ihmc.quadrupedRobotics.util;

import java.util.ArrayList;
import java.util.List;

/**
 * @see HeterogeneousMemoryPool
 */
public class HomogeneousMemoryPool<T>
{
   private final Class<T> type;
   private final List<T> elements;

   private int index = 0;

   public HomogeneousMemoryPool(Class<T> type)
   {
      this.type = type;
      this.elements = new ArrayList<>();
   }

   /**
    * @see HeterogeneousMemoryPool#lease(Class)
    */
   public T lease()
   {
      if (index >= elements.size())
      {
         try
         {
            elements.add(type.newInstance());
         }
         catch (InstantiationException | IllegalAccessException e)
         {
            e.printStackTrace();
         }
      }

      return elements.get(index++);
   }

   /**
    * @see HeterogeneousMemoryPool#evict()
    */
   public void evict()
   {
      index = 0;
   }
}
