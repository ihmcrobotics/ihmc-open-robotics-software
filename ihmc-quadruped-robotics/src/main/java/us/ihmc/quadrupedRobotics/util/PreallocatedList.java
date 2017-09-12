package us.ihmc.quadrupedRobotics.util;

import java.lang.reflect.Array;
import java.util.*;

public class PreallocatedList<E> implements List<E>
{
   protected final List<E> elements;
   protected final List<IntegerWrapper> indexes;
   protected IntegerWrapper size;

   public interface DefaultElementFactory<E>
   {
      E createDefaultElement();
   }

   public PreallocatedList(int capacity, final DefaultElementFactory<E> defaultElementFactory)
   {
      if (capacity < 0)
      {
         throw new RuntimeException("List capacity must be nonnegative.");
      }

      elements = new ArrayList<>();
      indexes = new ArrayList<>();
      for (int i = 0; i < capacity; i++)
      {
         elements.add(defaultElementFactory.createDefaultElement());
         indexes.add(new IntegerWrapper(i));
      }
      size = new IntegerWrapper(0);
   }

   public PreallocatedList(int capacity, final Class<E> element)
   {
      this(capacity, new DefaultElementFactory<E>()
      {
         @Override
         public E createDefaultElement()
         {
            try
            {
               return element.newInstance();
            }
            catch (InstantiationException | IllegalAccessException e)
            {
               throw new RuntimeException(e);
            }
         }
      });
   }

   public PreallocatedList(ArrayList<E> elements)
   {
      this.elements = new ArrayList<>();
      this.indexes = new ArrayList<>();
      for (int i = 0; i < elements.size(); i++)
      {
         this.elements.add(elements.get(i));
         this.indexes.add(new IntegerWrapper(i));
      }
      this.size = new IntegerWrapper(elements.size());
   }

   @Override
   public int size()
   {
      return size.getValue();
   }

   public int capacity()
   {
      return elements.size();
   }

   public void swap(int i1, int i2)
   {
      if ((i1 >= size.getValue()) || (i2 >= size.getValue()))
      {
         throw new IndexOutOfBoundsException();
      }
      swapUnsafe(i1, i2);
   }

   public boolean add()
   {
      return add(size.getValue());
   }

   public boolean add(int i)
   {
      if ((i < 0) || (i > size.getValue()))
      {
         throw new IndexOutOfBoundsException();
      }
      else if (size.getValue() == elements.size())
      {
         return false;
      }
      else
      {
         for (int j = size.getValue() - 1; j >= i; j--)
         {
            swapUnsafe(j, j + 1);
         }
         size.increment();
         return true;
      }
   }

   @Override
   public E get(int i)
   {
      if ((i < 0) || (i >= size.getValue()))
      {
         throw new IndexOutOfBoundsException();
      }
      else
      {
         return elements.get(indexes.get(i).getValue());
      }
   }

   @Override
   public E remove(int i)
   {
      if ((i < 0) || (i >= size.getValue()))
      {
         throw new IndexOutOfBoundsException();
      }
      else
      {
         for (int j = i; j < size.getValue() - 1; j++)
         {
            swapUnsafe(j, j + 1);
         }
         size.decrement();
         return null;
      }
   }

   @Override
   public boolean remove(Object o)
   {
      for (int i = 0; i < size.getValue(); i++)
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
      size.setValue(0);
   }

   @Override
   public boolean isEmpty()
   {
      return size.getValue() == 0;
   }

   @Override
   public boolean contains(Object o)
   {
      for (int index = 0; index < size.getValue(); index++)
      {
         if (get(index).equals(o))
            return true;
      }
      return false;
   }

   @Override
   public int indexOf(Object o)
   {
      for (int index = 0; index < size.getValue(); index++)
      {
         if (get(index).equals(o))
            return index;
      }
      return -1;
   }

   @Override
   public int lastIndexOf(Object o)
   {
      for (int index = size.getValue() - 1; index >= 0; index--)
      {
         if (get(index).equals(o))
            return index;
      }
      return -1;
   }

   @Override
   public Object[] toArray()
   {
      Object[] objects = new Object[size.getValue()];
      for (int i = 0; i < size.getValue(); i++)
      {
         objects[i] = get(i);
      }
      return objects;
   }

   @Override
   public <T> T[] toArray(T[] ts)
   {
      if (ts.length < size.getValue())
      {
         ts = (T[]) Array.newInstance(ts.getClass().getComponentType(), size.getValue());
      }
      else if (ts.length > size.getValue())
      {
         ts[size.getValue()] = null;
      }
      for (int i = 0; i < size.getValue(); i++)
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

   private void swapUnsafe(int i1, int i2)
   {
      int indexTmp = indexes.get(i1).getValue();
      indexes.get(i1).setValue(indexes.get(i2).getValue());
      indexes.get(i2).setValue(indexTmp);
   }

   protected class IntegerWrapper
   {
      private int value;

      public IntegerWrapper(int value)
      {
         this.value = value;
      }

      public int getValue()
      {
         return value;
      }

      public void setValue(int value)
      {
         this.value = value;
      }

      public void increment()
      {
         this.value++;
      }

      public void decrement()
      {
         this.value--;
      }
   }
}
