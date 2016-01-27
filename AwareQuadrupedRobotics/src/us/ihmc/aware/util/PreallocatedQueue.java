package us.ihmc.aware.util;

import java.util.ArrayList;
import java.util.List;

public class PreallocatedQueue<T>
{
   private final List<T> elements;
   private int position;
   private int size;

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
      return elements.get((position + queueIndex) % elements.size());
   }

   public T getTail()
   {
      if (size == 0)
      {
         return getHead();
      }
      else
      {
         return elements.get((position + size - 1) % elements.size());
      }
   }

   public T getHead()
   {
      return elements.get(position);
   }
}
