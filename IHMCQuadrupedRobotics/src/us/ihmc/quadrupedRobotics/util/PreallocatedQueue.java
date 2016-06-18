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

   public void resize(int size)
   {
      if (size > capacity)
      {
         throw new RuntimeException("Size must not exceed capacity");
      }
      this.size = size;
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

   public T get(int index)
   {
      if (index >= size)
      {
         throw new IndexOutOfBoundsException();
      }

      return elements.get((position + index) % elements.size());
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

   public void swap(int indexA, int indexB)
   {
      if (indexA >= size)
      {
         throw new IndexOutOfBoundsException();
      }
      if (indexB >= size)
      {
         throw new IndexOutOfBoundsException();
      }

      int a = (position + indexA) % elements.size();
      int b = (position + indexB) % elements.size();
      T tmp = elements.get(a);
      elements.set(a, elements.get(b));
      elements.set(b, tmp);
   }
}
