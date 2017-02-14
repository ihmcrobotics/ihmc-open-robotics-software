package us.ihmc.robotics.lists;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

import org.apache.commons.lang3.mutable.MutableInt;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.random.RandomTools;

public class RecyclingArrayListTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructor()
   {
      RecyclingArrayList<Object> list = new RecyclingArrayList<>(Object.class);
      assertTrue(list.isEmpty());
      assertTrue(list.size() == 0);
      assertTrue(list.getLast() == null);

      int expectedSize = 10;
      list = new RecyclingArrayList<>(expectedSize, Object.class);
      assertFalse(list.isEmpty());
      assertTrue(list.size() == expectedSize);
      assertTrue(list.getLast() != null);

      list.clear();
      assertTrue(list.isEmpty());
      assertTrue(list.size() == 0);
      assertTrue(list.getLast() == null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAddAndGet()
   {
      RecyclingArrayList<Object> list = new RecyclingArrayList<>(Object.class);
      ArrayList<Object> expectedList = new ArrayList<>();

      int finalSize = 10;
      for (int i = 0; i < finalSize; i++)
      {
         Object lastObject = list.add();
         expectedList.add(lastObject);
      }

      assertFalse(list.isEmpty());
      assertTrue(list.size() == finalSize);
      for (int i = 0; i < finalSize; i++)
      {
         assertTrue(list.get(i) == expectedList.get(i));
      }

      assertTrue(list.getLast() == expectedList.get(finalSize - 1));

      try
      {
         list.get(finalSize);
         fail();
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }

      list.clear();
      expectedList.clear();
      assertTrue(list.getLast() == null);

      finalSize = 8;
      for (int i = 0; i < finalSize; i++)
      {
         Object lastObject = list.add();
         expectedList.add(lastObject);
      }

      assertFalse(list.isEmpty());
      assertTrue(list.size() == finalSize);
      for (int i = 0; i < finalSize; i++)
      {
         assertTrue(list.get(i) == expectedList.get(i));
      }

      assertTrue(list.getLast() == expectedList.get(finalSize - 1));

      list.clear();
      expectedList.clear();
      assertTrue(list.getLast() == null);

      finalSize = 20;
      for (int i = 0; i < finalSize; i++)
      {
         Object lastObject = list.add();
         expectedList.add(lastObject);
      }

      assertFalse(list.isEmpty());
      assertTrue(list.size() == finalSize);
      for (int i = 0; i < finalSize; i++)
      {
         assertTrue(list.get(i) == expectedList.get(i));
      }

      assertTrue(list.getLast() == expectedList.get(finalSize - 1));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetAndGrowIfNeeded()
   {
      RecyclingArrayList<Object> list = new RecyclingArrayList<>(Object.class);

      assertTrue(list.isEmpty());
      assertTrue(list.size() == 0);

      int newSize = 10;
      Object lastObject = list.getAndGrowIfNeeded(newSize - 1);

      assertFalse(list.isEmpty());
      assertTrue(list.size() == newSize);

      for (int i = 0; i < newSize; i++)
      {
         assertTrue(list.get(i) != null);
         assertTrue(list.get(i) instanceof Object);
      }

      assertTrue(list.get(newSize - 1) == lastObject);
      assertTrue(list.getLast() == lastObject);

      int previousSize = newSize;
      newSize = 3;
      lastObject = list.getAndGrowIfNeeded(newSize - 1);

      assertFalse(list.isEmpty());
      assertTrue(list.size() == previousSize);

      for (int i = 0; i < newSize; i++)
      {
         assertTrue(list.get(i) != null);
         assertTrue(list.get(i) instanceof Object);
      }

      assertTrue(list.get(newSize - 1) == lastObject);
      assertTrue(list.getLast() == list.get(previousSize - 1));

      newSize = 13;
      lastObject = list.getAndGrowIfNeeded(newSize - 1);

      assertFalse(list.isEmpty());
      assertTrue(list.size() == newSize);

      for (int i = 0; i < newSize; i++)
      {
         assertTrue(list.get(i) != null);
         assertTrue(list.get(i) instanceof Object);
      }

      assertTrue(list.get(newSize - 1) == lastObject);
      assertTrue(list.getLast() == lastObject);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void growByOne()
   {
      RecyclingArrayList<Object> list = new RecyclingArrayList<>(Object.class);
      assertTrue(list.isEmpty());

      int currentSize = 0;
      assertTrue(list.size() == currentSize);

      list.growByOne();
      currentSize++;
      assertTrue(list.size() == currentSize);

      list.growByOne();
      currentSize++;
      assertTrue(list.size() == currentSize);

      for (int i = 0; i < currentSize; i++)
      {
         assertTrue(list.get(i) != null);
         assertTrue(list.get(i) instanceof Object);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void growByN()
   {
      RecyclingArrayList<Object> list = new RecyclingArrayList<>(Object.class);
      assertTrue(list.isEmpty());

      int currentSize = 0;
      assertTrue(list.size() == currentSize);

      list.growByN(15);
      currentSize += 15;
      assertTrue(list.size() == currentSize);

      list.growByN(1);
      currentSize += 1;
      assertTrue(list.size() == currentSize);

      for (int i = 0; i < currentSize; i++)
      {
         assertTrue(list.get(i) != null);
         assertTrue(list.get(i) instanceof Object);
      }

      try
      {
         list.growByN(-1);
         fail();
      }
      catch (RuntimeException e)
      {
         // Good
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFastRemove()
   {
      int currentSize = 10;
      RecyclingArrayList<Object> list = new RecyclingArrayList<>(currentSize, Object.class);
      assertTrue(list.size() == currentSize);

      ArrayList<Object> savedList = new ArrayList<>();
      for (int i = 0; i < currentSize; i++)
         savedList.add(list.get(i));

      int indexOfRemovedOject = 3;
      list.fastRemove(indexOfRemovedOject);
      currentSize--;
      assertTrue(list.size() == currentSize);

      for (int i = 0; i < currentSize; i++)
      {
         if (i == indexOfRemovedOject)
            assertTrue(list.get(i) == savedList.get(savedList.size() - 1));
         else
            assertTrue(list.get(i) == savedList.get(i));
      }

      try
      {
         list.fastRemove(currentSize);
         fail();
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRemove()
   {
      int currentSize = 10;
      RecyclingArrayList<MutableInt> list = new RecyclingArrayList<>(currentSize, MutableInt.class);
      for (int i = 0; i < currentSize; i++)
         list.get(i).setValue(10 + i);
      assertTrue(list.size() == currentSize);

      ArrayList<MutableInt> expectedList = new ArrayList<>();
      for (int i = 0; i < currentSize; i++)
         expectedList.add(list.get(i));

      int indexOfRemovedOject = 3;
      list.remove(indexOfRemovedOject);
      expectedList.remove(indexOfRemovedOject);
      currentSize--;
      assertTrue(list.size() == currentSize);

      for (int i = 0; i < currentSize; i++)
      {
         assertTrue(list.get(i) == expectedList.get(i));
      }

      indexOfRemovedOject = currentSize - 1;
      list.remove(indexOfRemovedOject);
      expectedList.remove(indexOfRemovedOject);
      currentSize--;
      assertTrue(list.size() == currentSize);

      for (int i = 0; i < currentSize; i++)
      {
         assertTrue(list.get(i) == expectedList.get(i));
      }

      try
      {
         list.remove(currentSize);
         fail();
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSwap()
   {
      Random rand = new Random(541964L);
      int currentSize = 10;
      RecyclingArrayList<MutableInt> list = new RecyclingArrayList<>(currentSize, MutableInt.class);
      for (int i = 0; i < currentSize; i++)
         list.get(i).setValue(10 + i);
      assertTrue(list.size() == currentSize);

      ArrayList<MutableInt> expectedList = new ArrayList<>();
      for (int i = 0; i < currentSize; i++)
         expectedList.add(list.get(i));

      for (int k = 0; k < 20; k++)
      {
         int indexA = RandomTools.generateRandomInt(rand, 0, currentSize - 1);
         int indexB = RandomTools.generateRandomInt(rand, 0, currentSize - 1);
         list.swap(indexA, indexB);
         Collections.swap(expectedList, indexA, indexB);
         assertTrue(list.size() == currentSize);

         for (int i = 0; i < currentSize; i++)
         {
            assertTrue(list.get(i) == expectedList.get(i));
         }
      }

      try
      {
         list.swap(0, currentSize);
         fail();
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }

      try
      {
         list.swap(currentSize, 0);
         fail();
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testInsertAtIndex()
   {
      Random rand = new Random(541964L);
      int currentSize = 10;
      RecyclingArrayList<MutableInt> list = new RecyclingArrayList<>(currentSize, MutableInt.class);
      for (int i = 0; i < currentSize; i++)
         list.get(i).setValue(10 + i);
      assertTrue(list.size() == currentSize);

      ArrayList<MutableInt> expectedList = new ArrayList<>();
      for (int i = 0; i < currentSize; i++)
         expectedList.add(list.get(i));

      for (int k = 0; k < 20; k++)
      {
         int randomIndex = RandomTools.generateRandomInt(rand, 0, currentSize);
         if (k == 5)
            randomIndex = currentSize;
         int newRandomValue = RandomTools.generateRandomInt(rand, 0, 52161);
         MutableInt newObject = list.insertAtIndex(randomIndex);
         newObject.setValue(newRandomValue);
         expectedList.add(randomIndex, newObject);
         currentSize++;
         assertTrue(list.size() == currentSize);

         for (int i = 0; i < currentSize; i++)
            assertTrue(list.get(i) == expectedList.get(i));
      }

      try
      {
         list.insertAtIndex(currentSize + 1);
         fail();
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testShuffle()
   {
      Random random = new Random(541964L);
      int currentSize = 100;
      RecyclingArrayList<MutableInt> list = new RecyclingArrayList<>(currentSize, MutableInt.class);

      for (int i = 0; i < currentSize; i++)
      {
         list.get(i).setValue(10 + i);
      }

      assertTrue(list.size() == currentSize);

      int sumBefore = 0;
      for (int i = 0; i < list.size; i++)
      {
         MutableInt value = list.get(i);
         sumBefore = sumBefore + value.intValue();
      }
      assertTrue(sumBefore > 0);

      list.shuffle(random);
      assertTrue(list.size() == currentSize);

      int sumAfter = 0;
      for (int i = 0; i < list.size; i++)
      {
         MutableInt value = list.get(i);
         sumAfter = sumAfter + value.intValue();
      }

      assertEquals(sumBefore, sumAfter);

   }
}
