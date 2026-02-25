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
   private int capacity;

   public GrowableRingIntBuffer(int initialCapacity)
   {
      tempValues = new TIntArrayList(initialCapacity);
      values = new TIntArrayList(initialCapacity);
      tempValues.clear();
      values.clear();
      capacity = initialCapacity;

      writeIndex = -1;
   }

   public void clear()
   {
      writeIndex = -1;
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
      unrollDataToTempHolder(values.size());

      switchActiveBuffers();

      // Change the write index to append to the end, since we grew.
      writeIndex = values.size() - 1;
   }

   private void shrinkValuesSize(int newCapacity)
   {
      // The capacity shrank.
      // we want to unroll the data from the values list into the pool array, and then copy the data from the pool back into the values list.
      // we want to ignore the oldest entries
      unrollDataToTempHolder(newCapacity);
      switchActiveBuffers();

      // The capacity shrank.
      if (values.size() >= capacity)
      {
         // The current size is greater than the new capacity, so start overwriting the oldest value, which is now the start since it was unrolled.
         writeIndex = -1;
      }
      else
      {
         writeIndex = values.size() - 1;
      }
   }

   private void unrollDataToTempHolder(int valuesToUnroll)
   {
      tempValues.clear();
      if (values.size() <= valuesToUnroll)
      {
         int readIndex = getStartOfWindowIndex();
         for (int i = 0; i < values.size(); i++)
         {
            tempValues.add(values.get(readIndex));
            readIndex = nextIndex(values.size(), readIndex);
         }
      }
      else
      {
         int valuesToIgnore = values.size() - valuesToUnroll;
         int readIndex = wrapIndex(getStartOfWindowIndex() + valuesToIgnore);
         for (int i = 0; i < valuesToUnroll; i++)
         {
            tempValues.add(values.get(readIndex));
            readIndex = nextIndex(values.size(), readIndex);
         }
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
      if (capacity == bufferCapacity)
      {
         // Get the next index
         writeIndex = nextIndex(bufferCapacity, writeIndex);
         // Grow the current size, up to a clamped amount.

         values.set(writeIndex, entry);
      }
      else
      {
         values.add(entry);
         writeIndex = values.size() - 1;
      }
   }

   public int getStart()
   {
      return values.get(getStartOfWindowIndex());
   }

   public int getEnd()
   {
      return values.get(writeIndex);
   }

   public int get(int entry)
   {
      return values.get(wrapIndex(getStartOfWindowIndex() + entry));
   }

   public int getCapacity()
   {
      return capacity;
   }

   public int getCurrentSize()
   {
      return values.size();
   }

   private static int nextIndex(int maxCapacity, int index)
   {
      if (index == maxCapacity - 1)
         return 0;
      return index + 1;
   }

   private int wrapIndex(int index)
   {
      while (index >= values.size())
         index -= values.size();

      return index;
   }

   private int getStartOfWindowIndex()
   {
      if (values.size() == capacity)
      {
         // the buffer is full, so get the next write index, as it's the start
         return nextIndex(values.size(), writeIndex);
      }
      else
      {
         // the buffer isn't full, so get the first field
         return 0;
      }
   }
}
