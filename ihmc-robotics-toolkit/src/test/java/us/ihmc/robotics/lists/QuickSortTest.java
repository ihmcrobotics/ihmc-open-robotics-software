package us.ihmc.robotics.lists;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertTrue;

public class QuickSortTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testSortListOfInteger() throws Exception
   {
      Random random = new Random(1516L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int size = random.nextInt(1000);
         List<Integer> list = new ArrayList<>();
         for (int index = 0; index < size; index++)
            list.add(random.nextBoolean() ? random.nextInt() : -random.nextInt());
         List<Integer> original = new ArrayList<>(list);

         Comparator<Integer> comparator = (o1, o2) -> o1.compareTo(o2);

         int startIndex = random.nextInt(size);
         int endIndex = random.nextInt(size - startIndex) + startIndex;
         QuickSort.sort(list, comparator, startIndex, endIndex);

         // Assert that before startIndex the list is unchanged
         for (int index = 0; index < startIndex; index++)
            assertTrue(list.get(index) == original.get(index));

         // Assert that the list has been sorted from startIndex to endIndex
         for (int index = startIndex; index < endIndex; index++)
            assertTrue(list.get(index) < list.get(index + 1));

         // Assert that after endIndex the list is unchanged
         for (int index = endIndex + 1; index < size; index++)
            assertTrue(list.get(index) == original.get(index));

      }
   }

   @Test
   public void testSortArrayOfInteger() throws Exception
   {
      Random random = new Random(1516L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int length = random.nextInt(1000);
         Integer[] array = new Integer[length];
         for (int index = 0; index < length; index++)
            array[index] = random.nextBoolean() ? random.nextInt() : -random.nextInt();
         Integer[] original = new Integer[length];
         System.arraycopy(array, 0, original, 0, length);

         Comparator<Integer> comparator = (o1, o2) -> o1.compareTo(o2);

         int startIndex = random.nextInt(length);
         int endIndex = random.nextInt(length - startIndex) + startIndex;
         QuickSort.sort(array, comparator, startIndex, endIndex);

         // Assert that before startIndex the array is unchanged
         for (int index = 0; index < startIndex; index++)
            assertTrue(array[index] == original[index]);

         // Assert that the array has been sorted from startIndex to endIndex
         for (int index = startIndex; index < endIndex; index++)
            assertTrue(array[index] < array[index + 1]);

         // Assert that after endIndex the list is unchanged
         for (int index = endIndex + 1; index < length; index++)
            assertTrue(array[index] == original[index]);
      }
   }
}
