package us.ihmc.quadrupedRobotics.util;

import java.util.*;

public class PreallocatedDeque<E> implements List<E>
{
   private int size;
   private int position;
   private final List<E> elements;
   private int capacity;

   public PreallocatedDeque(Class<E> element, int capacity)
   {
      size = 0;
      position = 0;
      elements = new ArrayList<>();
      for (int i = 0; i < capacity; i++)
      {
         try
         {
            elements.add(element.newInstance());
         }
         catch (InstantiationException | IllegalAccessException e)
         {
            e.printStackTrace();
         }
      }
      this.capacity = capacity;
   }

   public PreallocatedDeque(ArrayList<E> elements)
   {
      size = 0;
      position = 0;
      this.elements = elements;
   }

   public boolean pushBack()
   {
      if (size < elements.size())
      {
         size++;
         return true;
      }
      return false;
   }

   public boolean pushFront()
   {
      if (size < elements.size())
      {
         position = (position - 1) % elements.size();
         size++;
         return true;
      }
      return false;
   }

   public boolean popBack()
   {
      if (size > 0)
      {
         size--;
         return true;
      }
      return false;
   }

   public boolean popFront()
   {
      if (size > 0)
      {
         size--;
         position = (position + 1) % elements.size();
         return true;
      }
      return false;
   }

   public E back()
   {
      if (size == 0)
      {
         throw new IndexOutOfBoundsException();
      }

      return get(size - 1);
   }

   public E front()
   {
      if (size == 0)
      {
         throw new IndexOutOfBoundsException();
      }
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
      E tmp = elements.get(a);
      elements.set(a, elements.get(b));
      elements.set(b, tmp);
   }

   public int capacity()
   {
      return capacity;
   }

   @Override
   public int size()
   {
      return size;
   }

   @Override
   public E get(int index)
   {
      if (index >= size)
      {
         throw new IndexOutOfBoundsException();
      }

      return elements.get((position + index) % elements.size());
   }

   @Override
   public void clear()
   {
      size = 0;
   }

   @Override
   public boolean isEmpty()
   {
      return size == 0;
   }

   @Override
   public boolean contains(Object o)
   {
      for (int index = 0; index < size; index++)
      {
         if (get(index).equals(o))
            return true;
      }
      return false;
   }

   @Override
   public int indexOf(Object o)
   {
      for (int index = 0; index < size; index++)
      {
         if (get(index).equals(o))
            return index;
      }
      return -1;
   }

   @Override
   public int lastIndexOf(Object o)
   {
      for (int index = size - 1; index >= 0; index--)
      {
         if (get(index).equals(o))
            return index;
      }
      return -1;
   }

   @Override
   public Iterator<E> iterator()
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public Object[] toArray()
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public <T> T[] toArray(T[] ts)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public boolean remove(Object o)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public boolean containsAll(Collection<?> collection)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public boolean addAll(Collection<? extends E> collection)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public boolean removeAll(Collection<?> collection)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public boolean retainAll(Collection<?> collection)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public E set(int i, E e)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public boolean add(E e)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public void add(int index, E e)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public boolean addAll(int i, Collection<? extends E> collection)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public E remove(int i)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public ListIterator<E> listIterator()
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public ListIterator<E> listIterator(int i)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public List<E> subList(int i, int i1)
   {
      throw new UnsupportedOperationException();
   }
}
