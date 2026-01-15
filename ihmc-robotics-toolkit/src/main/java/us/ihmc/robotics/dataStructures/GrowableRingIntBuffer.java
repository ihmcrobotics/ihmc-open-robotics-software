package us.ihmc.robotics.dataStructures;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.interfaces.Settable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

public class GrowableRingIntBuffer
{
   private TIntArrayList tempValues = new TIntArrayList();
   private TIntArrayList values = new TIntArrayList();
   private int writeIndex;
   private int currentSize;
   private int capacity;

   public GrowableRingIntBuffer(int initialCapacity)
   {
      tempValues = new TIntArrayList(initialCapacity);
      values = new TIntArrayList(initialCapacity);
      tempValues.clear();
      values.clear();
      capacity = initialCapacity;

      currentSize = 0;
   }

   public void clear()
   {
      writeIndex = -1;
      currentSize = 0;
   }

   public void resize(int newCapacity)
   {
      if (newCapacity == capacity)
         return;

      if (newCapacity > capacity)
         growValuesSize();
      else
         shrinkValuesSize(newCapacity);

      capacity = newCapacity;
   }

   private void growValuesSize()
   {
      // The data pool grew. To make sure we're appending moving forward, we want to unroll the array.
      tempValues.clear();
      int readIndex = nextIndex(capacity, writeIndex);
      // copy the current values into the data pool, in order
      for (int i = 0; i < currentSize; i++)
      {
         tempValues.add(values.get(readIndex));
         readIndex = nextIndex(capacity, readIndex);
      }

      switchActiveBuffers();

      // Change the write index to append to the end, since we grew.
      writeIndex = currentSize;
   }

   private void shrinkValuesSize(int newCapacity)
   {
      // The capacity shrank.
      // we want to unroll the data from the values list into the pool array, and then copy the data from the pool back into the values list.
      // we want to ignore the oldest entries
      tempValues.clear();
      int readIndex = nextIndex(capacity, writeIndex + capacity - newCapacity);
      for (int i = 0; i < newCapacity; i++)
      {
         tempValues.add(values.get(readIndex));
         readIndex = nextIndex(capacity, readIndex);
      }

      switchActiveBuffers();

      // The capacity shrank.
      if (currentSize >= capacity)
      {
         // The current size is greater than the new capacity, so start overwriting the oldest value, which is now the start since it was unrolled.
         writeIndex = -1;
         currentSize = capacity;
      }
      else
      {
         writeIndex = currentSize;
      }
   }

   private void switchActiveBuffers()
   {
      TIntArrayList temp = values;
      values = tempValues;
      tempValues = temp;
   }

   public void add(int entry)
   {
      int bufferCapacity = values.size();

      // Get the next index
      writeIndex = nextIndex(bufferCapacity, writeIndex);
      // Grow the current size, up to a clamped amount.
      currentSize = Math.min(bufferCapacity, currentSize + 1);

      values.set(writeIndex, entry);
   }

   public int getStart()
   {
      return values.get(nextIndex(capacity, writeIndex));
   }

   public int getEnd()
   {
      return values.get(writeIndex);
   }

   public int get(int entry)
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
