package us.ihmc.robotics.lists;

import static org.junit.jupiter.api.Assertions.*;

import java.util.*;

import org.apache.commons.lang3.mutable.MutableInt;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;

public class RingBufferTest
{
   private static final int ITERATIONS = 1000;

   @Test
   public void testSimpleApplication()
   {
      Random random = new Random(43584756);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RingBuffer<MutableInt> buffer = new RingBuffer<>(random.nextInt(1500) + 1, MutableInt::new, MutableInt::setValue);
         List<MutableInt> elementsAdded = new ArrayList<>();

         for (int j = 0; j < 2.0 * buffer.capacity() + 100; j++)
         {
            MutableInt newElement = new MutableInt(j);
            elementsAdded.add(newElement);
            buffer.add(newElement);
            assertEquals(newElement, buffer.getLast());
            assertEquals(j >= buffer.capacity(), buffer.isBufferFull());

            if (!buffer.isBufferFull())
               assertEquals(elementsAdded.get(0), buffer.getFirst());
            else
               assertEquals(elementsAdded.get(j - buffer.capacity() + 1), buffer.getFirst());
         }

         List<MutableInt> ringBufferList = elementsAdded.subList(elementsAdded.size() - buffer.capacity(), elementsAdded.size());
         for (int j = 0; j < buffer.capacity(); j++)
         {
            assertEquals(ringBufferList.get(j), buffer.getFromFirst(j));
         }

         Collections.reverse(ringBufferList);
         for (int j = 0; j < buffer.capacity(); j++)
         {
            assertEquals(ringBufferList.get(j), buffer.getFromLast(j));
         }

         assertEquals(buffer.getFirst(), buffer.getFromFirst(0));
         assertEquals(buffer.getLast(), buffer.getFromFirst(buffer.capacity() - 1));
         assertEquals(buffer.getLast(), buffer.getFromLast(0));
         assertEquals(buffer.getFirst(), buffer.getFromLast(buffer.capacity() - 1));
      }
   }

   @Test
   public void testExampleWithPartiallyFilledBuffer()
   {
      Random random = new Random(8975);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RingBuffer<MutableInt> buffer = new RingBuffer<>(random.nextInt(1500) + 1, MutableInt::new, MutableInt::setValue);
         List<MutableInt> elementsAdded = new ArrayList<>();

         int numberOfElements = random.nextInt(buffer.capacity());

         for (int j = 0; j < numberOfElements; j++)
         {
            MutableInt newElement = new MutableInt(j);
            elementsAdded.add(newElement);
            buffer.add(newElement);
            assertEquals(newElement, buffer.getLast());
            assertFalse(buffer.isBufferFull());

            assertEquals(elementsAdded.get(0), buffer.getFirst());
            assertEquals(j + 1, buffer.size());
         }

         for (int j = 0; j < buffer.capacity(); j++)
         {
            if (j < buffer.size())
            {
               assertEquals(elementsAdded.get(j), buffer.getFromFirst(j));
            }
            else
            {
               int indexFinal = j;
               assertThrows(IndexOutOfBoundsException.class, () -> buffer.getFromFirst(indexFinal));
            }
         }

         Collections.reverse(elementsAdded);
         for (int j = 0; j < buffer.capacity(); j++)
         {
            if (j < buffer.size())
            {
               assertEquals(elementsAdded.get(j), buffer.getFromLast(j));
            }
            else
            {
               int indexFinal = j;
               assertThrows(IndexOutOfBoundsException.class, () -> buffer.getFromLast(indexFinal));
            }
         }
      }
   }

   @Test
   public void testToArray()
   {
      Random random = new Random(8975);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RingBuffer<MutableInt> buffer = new RingBuffer<>(random.nextInt(1500) + 1, MutableInt::new, MutableInt::setValue);
         List<MutableInt> elementsAdded = new ArrayList<>();

         int numberOfElements = random.nextInt(2 * buffer.capacity());

         for (int j = 0; j < numberOfElements; j++)
         {
            MutableInt newElement = new MutableInt(j);
            elementsAdded.add(newElement);
            buffer.add(newElement);
         }

         List<MutableInt> elementsInBuffer;

         if (buffer.isBufferFull())
            elementsInBuffer = elementsAdded.subList(elementsAdded.size() - buffer.capacity(), elementsAdded.size());
         else
            elementsInBuffer = elementsAdded;

         MutableInt[] expected = elementsInBuffer.toArray(new MutableInt[0]);
         MutableInt[] actual = buffer.toArrayFromFirstToLast();
         assertArrayEquals(expected, actual);

         Collections.reverse(Arrays.asList(expected));
         actual = buffer.toArrayFromLastToFirst();
         assertArrayEquals(expected, actual);
      }
   }

   @Test
   public void testChangeCapacity()
   {
      Random random = new Random(5645676);

      for (int i = 0; i < ITERATIONS; i++)
      { // Increasing the size.
         int initialCapacity = random.nextInt(1500) + 1;
         RingBuffer<MutableInt> buffer = new RingBuffer<>(initialCapacity, MutableInt::new, MutableInt::setValue);

         int numberOfElements = random.nextInt(2 * buffer.capacity());

         for (int j = 0; j < numberOfElements; j++)
         {
            MutableInt newElement = new MutableInt(j);
            buffer.add(newElement);
         }

         int sizeBeforeChangeCapacity = buffer.size();
         MutableInt[] expected = buffer.toArrayFromFirstToLast();

         int newCapacity = RandomNumbers.nextInt(random, initialCapacity + 1, 3 * initialCapacity);
         buffer.changeCapacity(newCapacity);

         assertFalse(buffer.isBufferFull());
         assertEquals(newCapacity, buffer.capacity());
         assertEquals(sizeBeforeChangeCapacity, buffer.size());

         MutableInt[] actual = buffer.toArrayFromFirstToLast();
         assertArrayEquals(expected, actual);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Decreasing the size.
         int initialCapacity = random.nextInt(1500) + 1;
         RingBuffer<MutableInt> buffer = new RingBuffer<>(initialCapacity, MutableInt::new, MutableInt::setValue);

         int numberOfElements = random.nextInt(2 * buffer.capacity());

         for (int j = 0; j < numberOfElements; j++)
         {
            MutableInt newElement = new MutableInt(j);
            buffer.add(newElement);
         }

         int sizeBeforeChangeCapacity = buffer.size();
         MutableInt[] expected = buffer.toArrayFromFirstToLast();

         int newCapacity = RandomNumbers.nextInt(random, 1, initialCapacity - 1);
         buffer.changeCapacity(newCapacity);

         assertEquals(sizeBeforeChangeCapacity >= newCapacity, buffer.isBufferFull());
         assertEquals(newCapacity, buffer.capacity());
         assertEquals(Math.min(sizeBeforeChangeCapacity, newCapacity), buffer.size());

         expected = Arrays.copyOfRange(expected, expected.length - Math.min(sizeBeforeChangeCapacity, newCapacity), expected.length);
         MutableInt[] actual = buffer.toArrayFromFirstToLast();
         assertArrayEquals(expected, actual);
      }
   }
}
