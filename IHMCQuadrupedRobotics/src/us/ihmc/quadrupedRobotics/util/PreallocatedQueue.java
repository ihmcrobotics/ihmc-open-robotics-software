package us.ihmc.quadrupedRobotics.util;

import java.util.ArrayList;
import java.util.List;

public class PreallocatedQueue<T>
{
   private int size;
   private int position;
   private final List<T> elements;
   private int capacity;

   public PreallocatedQueue(Class<T> type, int capacity)
   {
      size = 0;
      position = 0;
      elements = new ArrayList<>();
      for (int i = 0; i < capacity; i++)
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
      this.capacity = capacity;
   }

   public PreallocatedQueue(ArrayList<T> elements)
   {
      size = 0;
      position = 0;
      this.elements = elements;
   }

   public int size()
   {
      return size;
   }

   public int capacity()
   {
      return capacity;
   }

   public boolean enqueue()
   {
      if (size < elements.size())
      {
         size++;
         return true;
      }
      return false;
   }

   public boolean dequeue()
   {
      if (size > 0)
      {
         size--;
         position = (position + 1) % elements.size();
         return true;
      }
      return false;
   }

   public T get(int queueIndex)
   {
      if (queueIndex >= size)
      {
         throw new IndexOutOfBoundsException();
      }

      return elements.get((position + queueIndex) % elements.size());
   }

   public T getTail()
   {
      if (size == 0)
      {
         throw new IndexOutOfBoundsException();
      }

      return get(size - 1);
   }

   public T getHead()
   {
      return get(0);
   }
}
