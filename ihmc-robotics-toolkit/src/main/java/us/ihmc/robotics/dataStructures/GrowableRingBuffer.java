package us.ihmc.robotics.dataStructures;

import us.ihmc.euclid.interfaces.Settable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

public class GrowableRingBuffer<T extends Settable<T>>
{
   private final Supplier<T> allocator;
   private T[] dataPool;
   private final List<T> values = new ArrayList<>();
   private int writeIndex;
   private int currentSize;

   public GrowableRingBuffer(int initialCapacity, Supplier<T> allocator)
   {
      this.allocator = allocator;
      dataPool = (T[]) new Object[initialCapacity];
      for (int i = 0; i < initialCapacity; i++)
      {
         dataPool[i] = allocator.get();
         values.add(dataPool[i]);
      }

      currentSize = 0;
   }

   public void clear()
   {
      writeIndex = -1;
      currentSize = 0;
   }

   public void resize(int newCapacity)
   {
      if (newCapacity == values.size())
         return;

      if (newCapacity > dataPool.length)
         growPoolAndValuesSize(newCapacity);
      else if (newCapacity > values.size())
         growValuesSize(newCapacity);
      else
         shrinkValuesSize(newCapacity);
   }

   private void growPoolAndValuesSize(int newCapacity)
   {
      // Increase the data pool capacity
      ensureCapacity(newCapacity);
      // The data pool grew. We want to re-make the values array.
      int oldCapacity = values.size();
      // the data pool grew. We need to re-make the values array, looping through the old data in the data pool
      values.clear();
      int readIndex = nextIndex(oldCapacity, writeIndex);
      for (int i = 0; i < currentSize; i++)
      {
         values.add(dataPool[readIndex]);
         readIndex = nextIndex(oldCapacity, readIndex);
      }
      for (int i = currentSize; i < newCapacity; i++)
         values.add(dataPool[i]);

      // now make the data pool the same order, so that the backing data doesn't get fragmented
      for (int i = 0; i < currentSize; i++)
         dataPool[i] = values.get(i);

      // Change the write index to append to the end, since we grew.
      writeIndex = currentSize;
   }

   private void growValuesSize(int newCapacity)
   {
      // The data pool grew. We want to re-make the values array.
      int oldCapacity = values.size();
      // copy the current values into the data pool, in order
      for (int i = 0; i < currentSize; i++)
         dataPool[i] = values.get(i);
      // remake the values array
      values.clear();
      // Copy the backing, possibly looped data from the data pool back into the values array.
      int readIndex = nextIndex(oldCapacity, writeIndex);
      for (int i = 0; i < currentSize; i++)
      {
         values.add(dataPool[readIndex]);
         readIndex = nextIndex(oldCapacity, readIndex);
      }
      for (int i = currentSize; i < newCapacity; i++)
         values.add(dataPool[i]);

      // Change the write index to append to the end, since we grew.
      writeIndex = currentSize;
   }

   private void shrinkValuesSize(int newCapacity)
   {
      // The capacity shrank.
      // we want to unroll the data from the values list into the pool array, and then copy the data from the pool back into the values list.
      int oldCapacity = values.size();
      // we want to ignore the oldest entries
      int readIndex = nextIndex(oldCapacity, writeIndex + oldCapacity - newCapacity);
      for (int i = 0; i < currentSize; i++)
      {
         dataPool[i] = values.get(readIndex);
         readIndex = nextIndex(oldCapacity, readIndex);
      }
      values.clear();
      for (int i = 0; i < newCapacity; i++)
         values.add(dataPool[i]);

      // The capacity shrank.
      if (currentSize >= newCapacity)
      {
         // The current size is greater than the new capacity, so start overwriting the oldest value, which is now the start since it was unrolled.
         writeIndex = -1;
         currentSize = newCapacity;
      }
      else
      {
         writeIndex = currentSize;
      }
   }

   private void ensureCapacity(int minSize)
   {
      if (minSize <= dataPool.length)
         return;

      int previousArraySize = dataPool.length;
      dataPool = Arrays.copyOf(dataPool, minSize);

      for (int i = previousArraySize; i < minSize; i++)
      {
         dataPool[i] = allocator.get();
      }
   }

   public void add(T entry)
   {
      int bufferCapacity = values.size();

      // Get the next index
      writeIndex = nextIndex(bufferCapacity, writeIndex);
      // Grow the current size, up to a clamped amount.
      currentSize = Math.min(bufferCapacity, currentSize + 1);

      values.get(writeIndex).set(entry);
   }

   public T add()
   {
      int bufferCapacity = values.size();

      // Get the next index
      writeIndex = nextIndex(bufferCapacity, writeIndex);
      // Grow the current size, up to a clamped amount.
      currentSize = Math.min(bufferCapacity, currentSize + 1);

      return values.get(writeIndex);
   }

   public T get(int entry)
   {
      return values.get(wrapIndex(getStartOfWindowIndex(currentSize) + entry));
   }

   public int getCapacity()
   {
      return values.size();
   }

   public int getCurrentSize()
   {
      return currentSize;
   }

   private static int nextIndex(int maxCapacity, int index)
   {
      if (index == maxCapacity - 1)
         return 0;
      return index + 1;
   }

   private int wrapIndex(int index)
   {
      while (index > values.size())
         index -= values.size();

      return index;
   }

   private int getStartOfWindowIndex(int maxSize)
   {
      if (values.size() == maxSize)
      {
         // the buffer is full, so get the next write index, as it's the start
         return nextIndex(maxSize, writeIndex);
      }
      else
      {
         // the buffer isn't full, so get the first field
         return 0;
      }
   }
}
