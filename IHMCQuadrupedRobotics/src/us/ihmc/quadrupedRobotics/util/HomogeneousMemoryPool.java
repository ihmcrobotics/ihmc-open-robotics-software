package us.ihmc.quadrupedRobotics.util;

import java.util.ArrayList;
import java.util.List;

class HomogeneousMemoryPool<T>
{
   private final Class<T> type;
   private final List<T> elements;

   private int index = 0;

   public HomogeneousMemoryPool(Class<T> type)
   {
      this.type = type;
      this.elements = new ArrayList<>();
   }

   public T grab()
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

   public void reset()
   {
      index = 0;
   }
}
