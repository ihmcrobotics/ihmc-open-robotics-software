package us.ihmc.robotics.lists;

import org.apache.commons.lang3.math.NumberUtils;
import org.junit.jupiter.api.Test;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.Random;
import java.util.function.UnaryOperator;

import static us.ihmc.robotics.Assert.*;

public class YoPreallocatedListTest
{
   @Test
   public void testConstructor()
   {
      YoPreallocatedList<YoDouble> list = new YoPreallocatedList<>(YoDouble.class, "Test", 10, new YoRegistry("Test"));
      assertTrue(list.isEmpty());
      assertTrue(list.size() == 0);
      assertTrue(list.getLast() == null);
   }

   @Test
   public void testAddAndGet()
   {
      YoPreallocatedList<YoDouble> list = new YoPreallocatedList<>(YoDouble.class, "Test", 20, new YoRegistry("Test"));
      ArrayList<YoDouble> expectedList = new ArrayList<>();

      int finalSize = 10;
      for (int i = 0; i < finalSize; i++)
      {
         YoDouble lastObject = list.add();
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
         YoDouble lastObject = list.add();
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
         YoDouble lastObject = list.add();
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

   @Test
   public void testRemove()
   {
      YoPreallocatedList<YoInteger> list = new YoPreallocatedList<>(YoInteger.class, "Test", 10, new YoRegistry("Test"));
      int currentSize = 10;
      while (list.size() < currentSize)
         list.add().set(10 + list.size());

      ArrayList<YoInteger> expectedList = new ArrayList<>();
      for (int i = 0; i < currentSize; i++)
         expectedList.add(list.get(i));

      int indexOfRemovedObject = 3;
      list.remove(indexOfRemovedObject);
      expectedList.remove(indexOfRemovedObject);
      currentSize--;
      assertTrue(list.size() == currentSize);

      for (int i = 0; i < currentSize; i++)
      {
         assertTrue(list.get(i) == expectedList.get(i));
      }

      indexOfRemovedObject = currentSize - 1;
      list.remove(indexOfRemovedObject);
      expectedList.remove(indexOfRemovedObject);
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

   @Test
   public void testSwap()
   {
      Random rand = new Random(541964L);
      YoPreallocatedList<YoInteger> list = new YoPreallocatedList<>(YoInteger.class, "Test", 10, new YoRegistry("Test"));
      int currentSize = 10;
      while (list.size() < currentSize)
         list.add().set(10 + list.size());

      ArrayList<YoInteger> expectedList = new ArrayList<>();
      for (int i = 0; i < currentSize; i++)
         expectedList.add(list.get(i));

      for (int k = 0; k < 20; k++)
      {
         int indexA = rand.nextInt(currentSize);
         int indexB = rand.nextInt(currentSize);
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

   @Test
   public void testSort()
   {
      YoPreallocatedList<YoInteger> list = new YoPreallocatedList<>(YoInteger.class, "Test", 10, new YoRegistry("Test"));
      list.add().set(-3);
      list.add().set(20);
      list.add().set(-10);
      list.add().set(19);
      list.add().set(50);
      list.sort((o1, o2) -> NumberUtils.compare(o1.getIntegerValue(), o2.getIntegerValue()));
      assertTrue(list.get(0).getValue() == -10);
      assertTrue(list.get(1).getValue() == -3);
      assertTrue(list.get(2).getValue() == 19);
      assertTrue(list.get(3).getValue() == 20);
      assertTrue(list.get(4).getValue() == 50);
   }

   @Test
   public void testRemoveIndex()
   {
      int size = 10;
      YoPreallocatedList<YoInteger> list = new YoPreallocatedList<>(YoInteger.class, "Test", size, new YoRegistry("Test"));
      for (int i = 0; i < size; i++)
      {
         list.add().set(i);
      }

      assertTrue(list.remove(8).getValue() == 8);
      assertTrue(list.size() == size - 1);
      assertTrue(list.remove(4).getValue() == 4);
      assertTrue(list.size() == size - 2);
      assertTrue(list.remove(2).getValue() == 2);
      assertTrue(list.size() == size - 3);
      assertTrue(list.remove(size - 4).getValue() == size - 1);
      assertTrue(list.size() == size - 4);
   }

   @Test
   public void testUnsupportedOperations()
   {
      int size = 5;
      YoRegistry testRegistry = new YoRegistry("Test");
      YoDouble testDouble = new YoDouble("Test", testRegistry);

      YoPreallocatedList<YoDouble> list = new YoPreallocatedList<>(YoDouble.class, "Test", 10, testRegistry);
      for (int i = 0; i < size; i++)
      {
         list.add().set(i);
      }

      try
      {
         list.set(0, testDouble);
         fail();
      }
      catch(UnsupportedOperationException e)
      {
      }

      try
      {
         list.add(testDouble);
         fail();
      }
      catch(UnsupportedOperationException e)
      {
      }

      try
      {
         list.add(0, testDouble);
         fail();
      }
      catch(UnsupportedOperationException e)
      {
      }

      try
      {
         list.addAll(new HashSet<>());
         fail();
      }
      catch(UnsupportedOperationException e)
      {
      }

      try
      {
         list.replaceAll(UnaryOperator.identity());
         fail();
      }
      catch(UnsupportedOperationException e)
      {
      }

      try
      {
         list.iterator();
         fail();
      }
      catch(UnsupportedOperationException e)
      {
      }

      try
      {
         list.listIterator();
         fail();
      }
      catch(UnsupportedOperationException e)
      {
      }

      try
      {
         list.listIterator(0);
         fail();
      }
      catch(UnsupportedOperationException e)
      {
      }

      try
      {
         list.subList(0, 1);
         fail();
      }
      catch(UnsupportedOperationException e)
      {
      }
   }
}
