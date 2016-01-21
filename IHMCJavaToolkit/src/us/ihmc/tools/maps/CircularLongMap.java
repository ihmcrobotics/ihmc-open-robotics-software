package us.ihmc.tools.maps;

import java.util.Arrays;

public class CircularLongMap
{
   private int insertionIndex = 0;
   private int size = 0;

   private long lastInsertedKey = Long.MIN_VALUE;

   private final long[] keys;
   private final long[] values;

   public CircularLongMap(int elements)
   {
      keys = new long[elements];
      values = new long[elements];
   }

   /**
    * Insert a new key-value pair into the buffer. Keys have to be monotonically increasing (!)
    * 
    * @param key
    * @param value
    * 
    * @throws IndexOutOfBoundsException if key is smaller than or equal to the previously inserted key
    */
   public void insert(long key, long value)
   {
      if (key <= lastInsertedKey)
      {
         throw new IndexOutOfBoundsException("key is smaller than previously inserted key");
      }

      keys[insertionIndex] = key;
      values[insertionIndex] = value;

      lastInsertedKey = key;

      if (++insertionIndex >= keys.length)
      {
         insertionIndex = 0;
      }

      if (size < keys.length)
      {
         size++;
      }
   }

   public int size()
   {
      return size;
   }

   /**
    * Get 
    * 
    * @param matchNearest Match the nearest key if true.
    * @param key
    * @return
    */
   public long getValue(boolean matchNearest, long key)
   {
      int index;
      if (insertionIndex == 0)
      {
         index = Arrays.binarySearch(keys, 0, size, key);
         if (matchNearest && index == -1)
         {
            index = 0;
         }
      }
      else
      {
         if (size > insertionIndex && key < keys[0])
         {
            index = Arrays.binarySearch(keys, insertionIndex, size, key);
            if (matchNearest && index == -insertionIndex - 1)
            {
               index = insertionIndex;
            }
         }
         else
         {
            index = Arrays.binarySearch(keys, 0, insertionIndex, key);
            if (matchNearest && index == -1)
            {
               index = 0;
            }
         }
      }

      if (matchNearest)
      {
         return values[index < 0 ? -index - 2 : index];
      }
      else if (index < 0)
      {
         throw new IndexOutOfBoundsException("No key with value " + key + " found");
      }
      else
      {
         return values[index];
      }

   }

   public void print()
   {
      System.out.println("index: " + insertionIndex + " keys: " + Arrays.toString(keys));
   }
}
