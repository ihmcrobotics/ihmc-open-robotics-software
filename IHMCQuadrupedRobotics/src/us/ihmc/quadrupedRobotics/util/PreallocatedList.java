package us.ihmc.quadrupedRobotics.util;

import java.lang.reflect.Array;
import java.util.*;

public class PreallocatedList<E> implements List<E>
{
   private int size;
   private int position;
   private final List<E> elements;

   public PreallocatedList(Class<E> element, int capacity)
   {
      if (capacity < 0)
      {
         throw new RuntimeException("List capacity must be nonnegative.");
      }

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
   }

   public PreallocatedList(ArrayList<E> elements)
   {
      size = 0;
      position = 0;
      this.elements = elements;
   }

   @Override
   public int size()
   {
      return size;
   }

   public int capacity()
   {
      return elements.size();
   }

   public void swap(int indexA, int indexB)
   {
      if ((indexA >= size) || (indexB >= size))
      {
         throw new IndexOutOfBoundsException();
      }
      swapUnsafe(indexA, indexB);
   }

   public boolean add()
   {
      return add(size);
   }

   public boolean add(int i)
   {
      if ((i < 0) || (i > size))
      {
         throw new IndexOutOfBoundsException();
      }
      else if (size == elements.size())
      {
         return false;
      }
      else if (i < size / 2)
      {
         for (int j = 0; j < i; j++)
         {
            swapUnsafe(j, j - 1);
         }
         position = (position - 1) % elements.size();
         size++;
         return true;
      }
      else
      {
         for (int j = size - 1; j >= i; j--)
         {
            swapUnsafe(j, j + 1);
         }
         size++;
         return true;
      }
   }

   @Override
   public E get(int i)
   {
      if ((i < 0) || (i >= size))
      {
         throw new IndexOutOfBoundsException();
      }
      else
      {
         return elements.get((position + i) % elements.size());
      }
   }

   @Override
   public E remove(int i)
   {
      if ((i < 0) || (i >= size))
      {
         throw new IndexOutOfBoundsException();
      }
      else if (i < size / 2)
      {
         for (int j = i; j > 0; j--)
         {
            swapUnsafe(j, j - 1);
         }
         position = (position + 1) % elements.size();
         size--;
         return null;
      }
      else
      {
         for (int j = i; j < size - 1; j++)
         {
            swapUnsafe(j, j + 1);
         }
         size--;
         return null;
      }
   }

   @Override
   public boolean remove(Object o)
   {
      for (int i = 0; i < size; i++)
      {
         if (get(i).equals(o))
         {
            remove(i);
            return true;
         }
      }
      return false;
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
   public Object[] toArray()
   {
      Object[] objects = new Object[size];
      for (int i = 0; i < size; i++)
      {
         objects[i] = get(i);
      }
      return objects;
   }

   @Override
   public <T> T[] toArray(T[] ts)
   {
      if (ts.length < size)
      {
         ts = (T[]) Array.newInstance(ts.getClass().getComponentType(), size);
      }
      else if (ts.length > size)
      {
         ts[size] = null;
      }
      for (int i = 0; i < size; i++)
      {
         ts[i] = (T)get(i);
      }
      return ts;
   }

   @Override
   public boolean add(E e)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public void add(int i, E e)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public boolean addAll(int i, Collection<? extends E> collection)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public boolean addAll(Collection<? extends E> collection)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public E set(int i, E e)
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
   public boolean containsAll(Collection<?> collection)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public Iterator<E> iterator()
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

   private void swapUnsafe(int indexA, int indexB)
   {
      int a = (position + indexA) % elements.size();
      int b = (position + indexB) % elements.size();
      E tmp = elements.get(a);
      elements.set(a, elements.get(b));
      elements.set(b, tmp);
   }
}
