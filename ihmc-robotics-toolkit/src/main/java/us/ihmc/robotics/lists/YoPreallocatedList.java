package us.ihmc.robotics.lists;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

import java.lang.reflect.Array;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.*;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

/**
 * YoVariablized version of the PreallocatedList from ihmc-commons
 */
public class YoPreallocatedList<T> implements List<T>
{
   private final Class<T> clazz;
   private final T[] values;
   private final YoInteger position;

   public YoPreallocatedList(Class<T> clazz, String prefix, int capacity, YoRegistry registry)
   {
      this(clazz, new DefaultYoVariableAllocator<>(clazz, prefix, registry), prefix, registry, capacity);
   }

   public YoPreallocatedList(Class<T> clazz, Supplier<T> allocator, String prefix, YoRegistry registry, int capacity)
   {
      this.clazz = clazz;
      this.position = new YoInteger(prefix + "Index", registry);
      this.values = (T[]) Array.newInstance(clazz, capacity);

      for (int i = 0; i < capacity; i++)
      {
         values[i] = allocator.get();
      }

      position.set(-1);
   }

   public YoInteger getYoPosition()
   {
      return position;
   }

   /**
    * Returns the elements in this list as array
    *
    * This method allocates a new array
    *
    * @return new array of length size();
    */
   @Override
   public T[] toArray()
   {
      @SuppressWarnings("unchecked")
      T[] array = (T[]) Array.newInstance(clazz, size());
      System.arraycopy(values, 0, array, 0, size());
      return array;
   }

   /** {@inheritDoc} */
   @Override
   @SuppressWarnings("unchecked")
   public <S> S[] toArray(S[] dest)
   {
      int size = size();
      if (dest.length < size)
      {
         return (S[]) Arrays.copyOf(values, size, dest.getClass());
      }
      System.arraycopy(values, 0, dest, 0, size);
      if (dest.length > size)
         dest[size] = null;
      return dest;
   }

   /**
    * Clears the list.
    *
    * This function just resets the size to 0. The underlying data objects are not emptied or removed.
    */
   public void resetQuick()
   {
      position.set(-1);
   }

   /**
    * Add a value and return a handle to the object.
    *
    * @return value at the last position. This object can still hold data.
    */
   public T add()
   {
      maxCapacityCheck(position.getIntegerValue() + 1);
      position.increment();
      T val = values[position.getIntegerValue()];
      return val;
   }

   /**
    * Removes the last element in the list. The underlying data object is not emptied or removed
    */
   public void remove()
   {
      nonEmptyCheck();
      position.decrement();
   }

   /**
    * Removes the element at the specified position in this list.
    * Shifts any subsequent elements to the left (subtracts one from their
    * indices).
    *
    * @param i the index of the element to be removed
    */
   @Override
   public T remove(int i)
   {
      if (i == position.getIntegerValue())
      {
         remove();
         return values[i];
      }

      rangeCheck(i);

      T t = values[i];

      while (i < position.getIntegerValue())
      {
         values[i] = values[++i];
      }

      // Do not throw away the removed element, put it at the end of the list instead.
      values[position.getIntegerValue()] = t;
      position.decrement();
      return t;
   }

   /**
    * Swap two objects of this list.
    *
    * @param i index of the first object to swap
    * @param j index of the second object to swap
    * @throws ArrayIndexOutOfBoundsException if either of the indices is out of range
    *            (<tt>i &lt; 0 || i &gt;= size() || j &lt; 0 || j &gt;= size()</tt>)
    */
   public void swap(int i, int j)
   {
      rangeCheck(i);
      rangeCheck(j);

      if (i == j)
      {
         return;
      }

      unsafeSwap(i, j);
   }

   /**
    * Sorts the array in place using {@link Arrays::sort}
    * @param comparator to determine element ordering
    */
   @Override
   public void sort(Comparator<? super T> comparator)
   {
      if(size() == 0)
         return;
      Arrays.sort(values, 0, size(), comparator);
   }

   private void unsafeSwap(int i, int j)
   {
      T t = values[i];
      values[i] = values[j];
      values[j] = t;
   }

   /**
    * Get the element at position i. To change the element, use get() and
    *
    * @param i Position to get element at
    * @return Element at position i.
    */
   @Override
   public T get(int i)
   {
      rangeCheck(i);
      return values[i];
   }

   /**
    * Returns the first element of this list. If the list is empty, it returns {@code null}.
    *
    * @return the first element of this list.
    */
   public T getFirst()
   {
      if (isEmpty())
      {
         return null;
      }
      else
      {
         return values[0];
      }
   }

   /**
    * Returns the last element of this list. If the list is empty, it returns {@code null}.
    *
    * @return the last element of this list.
    */
   public T getLast()
   {
      if (isEmpty())
      {
         return null;
      }
      else
      {
         return values[position.getIntegerValue()];
      }
   }

   /**
    * Clears the list
    *
    * This function just resets the size to 0.
    *
    * The underlying data objects are not emptied or removed, however this may change in future
    * releases
    *
    */
   @Override
   public void clear()
   {
      resetQuick();
   }

   /**
    * Returns the number of active elements in this list
    */
   @Override
   public int size()
   {
      return position.getIntegerValue() + 1;
   }

   /**
    * Returns {@code true} if this list contains no elements.
    *
    * @return {@code true} if this list contains no elements.
    */
   @Override
   public boolean isEmpty()
   {
      return size() == 0;
   }

   /**
    * @return the maximum capacity of this list
    */
   public int capacity()
   {
      return values.length;
   }

   /**
    * @return the remaining space in this sequence (capacity() - size())
    */
   public int remaining()
   {
      return capacity() - size();
   }

   private void nonEmptyCheck()
   {
      if (position.getIntegerValue() < 0)
      {
         throw new ArrayIndexOutOfBoundsException("List is empty");
      }
   }

   private void rangeCheck(int index)
   {
      if(index < 0 || index > this.position.getIntegerValue())
      {
         throw new ArrayIndexOutOfBoundsException("Position is not valid in the list, size is " + size() + ", requested element is " + index);
      }
   }

   private void maxCapacityCheck(int newSize)
   {
      if (newSize >= this.values.length)
      {
         throw new ArrayIndexOutOfBoundsException("Cannot add element to sequence, max size is violated");
      }
   }

   /**
    * Hashcode computed from the size of the array,
    * and respective hashcodes of the current data.
    *
    * @return hashCode for this list
    * @see Arrays#hashCode(Object[])
    */
   @Override
   public int hashCode()
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + position.getIntegerValue();
      result = prime * result + 1237;
      result = prime * result + Arrays.hashCode(values);
      return result;
   }

   /** {@inheritDoc} */
   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (!(obj instanceof List))
         return false;
      List<?> other = (List<?>) obj;
      if (size() != other.size())
         return false;
      for (int i = 0; i < size(); i++)
      {
         if (!values[i].equals(other.get(i)))
            return false;
      }
      return true;
   }

   @Override
   public String toString()
   {
      String s = "";
      s += clazz.getSimpleName();
      s += " pos: " + position.getIntegerValue();
      s += " [";
      for (int i = 0; i < size(); i++)
      {
         if (i > 0)
            s += ", ";
         s += values[i].toString();
      }
      s += "]";
      return s;
   }


   /** {@inheritDoc} */
   @Override
   public boolean contains(Object o)
   {
      for (int i = 0; i < size(); i++)
      {
         if(values[i].equals(o))
         {
            return true;
         }
      }

      return false;
   }

   /** {@inheritDoc} */
   @Override
   public boolean containsAll(Collection<?> c)
   {
      for (Object o : c)
      {
         if (!contains(o))
            return false;
      }
      return true;
   }

   private static class DefaultYoVariableAllocator<S> implements Supplier<S>
   {
      private final Constructor<S> constructor;
      private final String prefix;
      private final YoRegistry registry;
      private int index = 0;

      public DefaultYoVariableAllocator(Class<S> clazz, String prefix, YoRegistry registry)
      {
         this.prefix = prefix;
         this.registry = registry;

         try
         {
            constructor = clazz.getConstructor(String.class, registry.getClass());
         }
         catch(NoSuchMethodException e)
         {
            throw new RuntimeException("Could not find (String, YoRegistry) constructor for class " + clazz.getSimpleName());
         }
      }

      @Override
      public S get()
      {
         try
         {
            return constructor.newInstance(prefix + index++, registry);
         }
         catch(InstantiationException | IllegalAccessException | InvocationTargetException e)
         {
            throw new RuntimeException("Could not call constructor");
         }
      }
   }


   // Unsupported operations

   /**
    * Always throws an {@code UnsupportedOperationException}.
    * Set elements by calling {@link #get(int)} or {@link #add()}
    * and operating on the returned object
    */
   @Override
   public T set(int index, T element)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Always throws an {@code UnsupportedOperationException}.
    * Add elements by calling {@link #add()} and operating on the returned object
    */
   @Override
   public boolean add(T t)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Always throws an {@code UnsupportedOperationException}.
    * Set elements by calling {@link #get(int)} or {@link #add()}
    * and operating on the returned object
    */
   @Override
   public void add(int index, T element)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Always throws an {@code UnsupportedOperationException}.
    * Add elements by calling {@link #add()} and operating on the returned object
    */
   @Override
   public boolean addAll(Collection<? extends T> c)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Always throws an {@code UnsupportedOperationException}.
    * Add elements by calling {@link #add()} and operating on the returned object
    */
   @Override
   public boolean addAll(int index, Collection<? extends T> c)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Always throws an {@code UnsupportedOperationException}.
    * Set elements by calling {@link #get(int)} and operating on the returned object
    */
   @Override
   public void replaceAll(UnaryOperator<T> operator)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Always throws an {@code UnsupportedOperationException}.
    * Iterate using indices and not an iterator
    */
   @Override
   public Iterator<T> iterator()
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Always throws an {@code UnsupportedOperationException}.
    * Iterate using indices and not an iterator
    */
   @Override
   public ListIterator<T> listIterator()
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Always throws an {@code UnsupportedOperationException}.
    * Iterate using indices and not an iterator
    */
   @Override
   public ListIterator<T> listIterator(int index)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Always throws an {@code UnsupportedOperationException}.
    */
   @Override
   public List<T> subList(int fromIndex, int toIndex)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Throws an UnsupportedOperationException. YoVariable primitives don't override .equals
    */
   @Override
   public int indexOf(Object o)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Throws an UnsupportedOperationException. YoVariable primitives don't override .equals
    */
   @Override
   public int lastIndexOf(Object o)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Throws an UnsupportedOperationException. YoVariable primitives don't override .equals
    */
   @Override
   public boolean remove(Object o)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Throws an UnsupportedOperationException. YoVariable primitives don't override .equals
    */
   @Override
   public boolean removeAll(Collection<?> c)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Throws an UnsupportedOperationException. YoVariable primitives don't override .equals
    */
   @Override
   public boolean retainAll(Collection<?> c)
   {
      throw new UnsupportedOperationException();
   }
}
