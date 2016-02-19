package us.ihmc.robotics.lists;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.Arrays;

public class RecyclingArrayList<T>
{
   /**
    * Default initial capacity.
    */
   private static final int DEFAULT_INITIAL_SIZE = 0;

   private T[] elementData;
   private final Builder<T> builder;
   protected int size = 0;

   public RecyclingArrayList(Class<T> clazz)
   {
      this(DEFAULT_INITIAL_SIZE, createBuilderWithEmptyConstructor(clazz));
   }

   public RecyclingArrayList(Builder<T> builder)
   {
      this(DEFAULT_INITIAL_SIZE, builder);
   }

   public RecyclingArrayList(int initialCapacity, Class<T> clazz)
   {
      this(initialCapacity, createBuilderWithEmptyConstructor(clazz));
   }

   @SuppressWarnings("unchecked")
   public RecyclingArrayList(int initialCapacity, Builder<T> builder)
   {
      elementData = (T[]) new Object[initialCapacity];
      size = initialCapacity;
      this.builder = builder;

      fillElementDataIfNeeded();
      clear();
   }

   public int size()
   {
      return size;
   }

   public boolean isEmpty()
   {
      return size == 0;
   }

   public void clear()
   {
      size = 0;
   }

   public T add()
   {
      return getAndGrowIfNeeded(size);
   }

   public T insertAtIndex(int i)
   {
      rangeCheckForInsert(i);

      // First add new element at last index
      T ret = add();

      // Then go trough the list by swapping elements two by two to reach the desired index
      for (int index = size - 1; index > i; index--)
         unsafeFastSwap(index, index - 1);

      return ret;
   }

   public T get(int i)
   {
      rangeCheck(i);
      return unsafeGet(i);
   }

   public T getLast()
   {
      if (isEmpty())
         return null;
      else
         return unsafeGet(size - 1);
   }

   protected T unsafeGet(int i)
   {
      return elementData[i];
   }

   public T getAndGrowIfNeeded(int i)
   {
      if (i >= size)
      {
         size = i + 1;
         ensureCapacity(size);
      }

      return elementData[i];
   }

   public void growByOne()
   {
      unsafeGrowByN(1);
   }

   public void growByN(int n)
   {
      if (n == 0)
         return;
      else if (n < 0)
         throw new RuntimeException("Cannot grow the list by a negative number. Given number for growing list:" + n);
      unsafeGrowByN(n);
   }

   protected void unsafeGrowByN(int n)
   {
      size += n;
      ensureCapacity(size);
   }

   /**
    * Removes the element at the specified position in this list.
    * This method is faster than {@code RecyclingArrayList#remove(int)} but the ith element is swapped with the last element changing the ordering of the list.
    * 
    * @param index the index of the element to be removed
    */
   public void fastRemove(int i)
   {
      if (i == size - 1)
      {
         size--;
         return;
      }
      rangeCheck(i);
      unsafeFastSwap(i, --size);
   }

   public void swap(int i, int j)
   {
      rangeCheck(i);
      rangeCheck(j);

      unsafeSwap(i, j);
   }

   protected void unsafeSwap(int i, int j)
   {
      if (i == j) return;

      unsafeFastSwap(i, j);
   }

   private void unsafeFastSwap(int i, int j)
   {
      T t = elementData[i];
      elementData[i] = elementData[j];
      elementData[j] = t;
   }

   /**
    * Removes the element at the specified position in this list.
    * Shifts any subsequent elements to the left (subtracts one from their
    * indices).
    * 
    * @param index the index of the element to be removed
    */
   public void remove(int i)
   {
      if (i == size - 1)
      {
         size--;
         return;
      }
      rangeCheck(i);

      T t = elementData[i];

      while (i < size - 1)
      {
         elementData[i] = elementData[++i];
      }

      // Do not throw away the removed element, put it at the end of the list instead.
      elementData[size - 1] = t;
      size--;
   }

   protected void ensureCapacity(int minCapacity)
   {
      if (minCapacity <= elementData.length)
         return;

      int previousArraySize = elementData.length;
      elementData = Arrays.copyOf(elementData, minCapacity);

      for (int i = previousArraySize; i < minCapacity; i++)
      {
         elementData[i] = builder.newInstance();
      }
   }

   private void fillElementDataIfNeeded()
   {
      for (int i = 0; i < elementData.length; i++)
      {
         if (elementData[i] == null)
            elementData[i] = builder.newInstance();
      }
   }

   /**
    * Checks if the given index is in range.  If not, throws an appropriate
    * runtime exception.  This method does *not* check if the index is
    * negative: It is always used immediately prior to an array access,
    * which throws an ArrayIndexOutOfBoundsException if index is negative.
    */
   protected void rangeCheck(int index)
   {
      if (index >= size)
         throw new IndexOutOfBoundsException("Index: " + index + ", Size: " + size);
   }

   protected void rangeCheckForInsert(int index)
   {
      if (index > size)
         throw new IndexOutOfBoundsException("Index: " + index + ", Size: " + size);
   }

   public static abstract class Builder<V>
   {
      public abstract V newInstance();
   }

   @Override
   public String toString()
   {
      if (isEmpty())
         return "Empty list";

      String ret = "";

      for (int i = 0; i < size - 1; i++)
         ret += unsafeGet(i).toString() + "\n";
      ret += unsafeGet(size - 1).toString();

      return ret;
   }

   private static <U> Builder<U> createBuilderWithEmptyConstructor(Class<U> clazz)
   {
      final Constructor<U> emptyConstructor;
      // Trying to get an empty constructor from clazz
      try
      {
         emptyConstructor = clazz.getConstructor();
      }
      catch (NoSuchMethodException | SecurityException e)
      {
         throw new RuntimeException("Could not find a visible empty constructor in the class: " + clazz.getSimpleName());
      }

      Builder<U> builder = new Builder<U>()
      {
         @Override
         public U newInstance()
         {
            U newInstance = null;

            try
            {
               newInstance = emptyConstructor.newInstance();
            }
            catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
            {
               e.printStackTrace();
               throw new RuntimeException("Something went wrong the empty constructor implemented in the class: " + emptyConstructor.getDeclaringClass().getSimpleName());
            }

            return newInstance;
         }
      };
      return builder;
   }
}
