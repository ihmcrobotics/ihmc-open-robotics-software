package us.ihmc.robotics.lists;

import static org.junit.Assert.*;
import static us.ihmc.robotics.lists.ListWrappingIndexTools.*;

import java.io.IOException;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.robotics.lists.ListWrappingIndexTools;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.MutationTestingTools;

public class ListWrappingIndexToolsTest
{
   @ContinuousIntegrationTest(estimatedDuration =  0.0)
   @Test(timeout = 10000)
   public void testWrap()
   {
      Random random = new Random(234223L);

      for (int iteration = 0; iteration < 100; iteration++)
      {
         int listSize = 1 + random.nextInt(20);
         List<Object> list = new ArrayList<>();
         for (int i = 0; i < listSize; i++)
            list.add(new Object());

         for (int index = 0; index < listSize; index++)
            assertEquals(index, wrap(index, list));
         for (int index = 0; index < listSize; index++)
            assertEquals(index, wrap(index - listSize, list));
         for (int index = 0; index < listSize; index++)
            assertEquals(index, wrap(index + listSize, list));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration =  0.0)
   @Test(timeout = 10000)
   public void testNext()
   {
      Random random = new Random(234223L);

      for (int iteration = 0; iteration < 100; iteration++)
      {
         int listSize = 1 + random.nextInt(20);
         List<Object> list = new ArrayList<>();
         for (int i = 0; i < listSize; i++)
            list.add(new Object());

         for (int index = 0; index < listSize; index++)
            assertEquals(wrap(index + 1, list), next(index, list));
         for (int index = 0; index < listSize; index++)
            assertEquals(wrap(index + 1, list), next(index - listSize, list));
         for (int index = 0; index < listSize; index++)
            assertEquals(wrap(index + 1, list), next(index + listSize, list));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration =  0.0)
   @Test(timeout = 10000)
   public void testDecrement()
   {
      Random random = new Random(234223L);

      for (int iteration = 0; iteration < 100; iteration++)
      {
         int listSize = 1 + random.nextInt(20);
         List<Object> list = new ArrayList<>();
         for (int i = 0; i < listSize; i++)
            list.add(new Object());

         for (int index = 0; index < listSize; index++)
            assertEquals(wrap(index - 1, list), previous(index, list));
         for (int index = 0; index < listSize; index++)
            assertEquals(wrap(index - 1, list), previous(index - listSize, list));
         for (int index = 0; index < listSize; index++)
            assertEquals(wrap(index - 1, list), previous(index + listSize, list));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration =  0.0)
   @Test(timeout = 10000)
   public void testGetWrap()
   {
      Random random = new Random(234223L);

      for (int iteration = 0; iteration < 100; iteration++)
      {
         int listSize = 1 + random.nextInt(20);
         List<Object> list = new ArrayList<>();
         for (int i = 0; i < listSize; i++)
            list.add(new Object());

         for (int index = 0; index < listSize; index++)
            assertEquals(list.get(index), getWrap(index, list));
         for (int index = 0; index < listSize; index++)
            assertEquals(list.get(index), getWrap(index - listSize, list));
         for (int index = 0; index < listSize; index++)
            assertEquals(list.get(index), getWrap(index + listSize, list));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration =  0.0)
   @Test(timeout = 10000)
   public void testGetNextWrap()
   {
      Random random = new Random(234223L);

      for (int iteration = 0; iteration < 100; iteration++)
      {
         int listSize = 1 + random.nextInt(20);
         List<Object> list = new ArrayList<>();
         for (int i = 0; i < listSize; i++)
            list.add(new Object());

         for (int index = 0; index < listSize; index++)
            assertEquals(list.get(next(index, list)), getNext(index, list));
         for (int index = 0; index < listSize; index++)
            assertEquals(list.get(next(index, list)), getNext(index - listSize, list));
         for (int index = 0; index < listSize; index++)
            assertEquals(list.get(next(index, list)), getNext(index + listSize, list));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration =  0.0)
   @Test(timeout = 10000)
   public void testGetPreviousWrap()
   {
      Random random = new Random(234223L);

      for (int iteration = 0; iteration < 100; iteration++)
      {
         int listSize = 1 + random.nextInt(20);
         List<Object> list = new ArrayList<>();
         for (int i = 0; i < listSize; i++)
            list.add(new Object());

         for (int index = 0; index < listSize; index++)
            assertEquals(list.get(previous(index, list)), getPrevious(index, list));
         for (int index = 0; index < listSize; index++)
            assertEquals(list.get(previous(index, list)), getPrevious(index - listSize, list));
         for (int index = 0; index < listSize; index++)
            assertEquals(list.get(previous(index, list)), getPrevious(index + listSize, list));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration =  0.0)
   @Test(timeout = 10000)
   public void testSubLengthInclusive()
   {
      int listSize = 20;
      List<Object> list = new ArrayList<>();
      for (int i = 0; i < listSize; i++)
         list.add(new Object());

      for (int index = 0; index < listSize; index++)
         assertEquals(1, subLengthInclusive(index, index, list));
      for (int index = 0; index < listSize; index++)
      {
         int startIndex = index;
         int endIndex = index + listSize - 1;
         assertEquals(listSize, subLengthInclusive(startIndex, endIndex, list));
         assertEquals(listSize, subLengthInclusive(startIndex, wrap(endIndex, list), list));
      }

      for (int index = 0; index < listSize; index++)
      {
         int startIndex = index;
         int endIndex = index + listSize / 2 - 1;
         assertEquals(listSize / 2, subLengthInclusive(startIndex, endIndex, list));
         assertEquals(listSize / 2, subLengthInclusive(startIndex, wrap(endIndex, list), list));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration =  0.0)
   @Test(timeout = 10000)
   public void testSubLengthExclusive()
   {
      int listSize = 20;
      List<Object> list = new ArrayList<>();
      for (int i = 0; i < listSize; i++)
         list.add(new Object());

      for (int index = 0; index < listSize; index++)
         assertEquals(0, subLengthExclusive(index, index, list));
      for (int index = 0; index < listSize; index++)
      {
         int startIndex = index;
         int endIndex = index + listSize + 1;
         assertEquals(listSize, subLengthExclusive(startIndex, endIndex, list));
         // Once wrapped, we lose the information about endIndex - startIndex = listSize, which is necessary to understand that the user wants the entire list.
//         assertEquals(listSize, subLengthExclusive(startIndex, wrap(endIndex, list), list));
      }

      for (int index = 0; index < listSize; index++)
      {
         int startIndex = index;
         int endIndex = index + listSize / 2 + 1;
         assertEquals(listSize / 2, subLengthExclusive(startIndex, endIndex, list));
         assertEquals(listSize / 2, subLengthExclusive(startIndex, wrap(endIndex, list), list));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration =  0.0)
   @Test(timeout = 10000)
   public void testSubListInclusive()
   {
      int listSize = 20;
      List<Object> list = new ArrayList<>();
      for (int i = 0; i < listSize; i++)
         list.add(new Object());

      for (int index = 0; index < listSize; index++)
      {
         List<Object> subListInclusive = subListInclusive(index, index, list);
         assertEquals(1, subListInclusive.size());
         assertEquals(list.get(index), subListInclusive.get(0));
      }

      for (int index = 0; index < listSize; index++)
      {
         int startIndex = index;
         int endIndex = index + listSize - 1;
         List<Object> subListInclusive = subListInclusive(startIndex, endIndex, list);
         assertEquals(subLengthInclusive(startIndex, endIndex, list), subListInclusive.size());

         for (int i = startIndex; i <= endIndex; i++)
            assertEquals(getWrap(i, list), subListInclusive.get(i - startIndex));
      }

      for (int index = 0; index < listSize; index++)
      {
         int startIndex = index;
         int endIndex = index + listSize / 2 - 1;
         List<Object> subListInclusive = subListInclusive(startIndex, endIndex, list);
         assertEquals(subLengthInclusive(startIndex, endIndex, list), subListInclusive.size());

         for (int i = startIndex; i <= endIndex; i++)
            assertEquals(getWrap(i, list), subListInclusive.get(i - startIndex));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration =  0.0)
   @Test(timeout = 10000)
   public void testSubListExclusive()
   {
      int listSize = 20;
      List<Object> list = new ArrayList<>();
      for (int i = 0; i < listSize; i++)
         list.add(new Object());

      for (int index = 0; index < listSize; index++)
      {
         List<Object> subListExclusive = subListExclusive(index, index, list);
         assertEquals(0, subListExclusive.size());
      }

      for (int index = 0; index < listSize; index++)
      {
         int startIndex = index;
         int endIndex = index + listSize + 1;
         List<Object> subListExclusive = subListExclusive(startIndex, endIndex, list);
         assertEquals(subLengthExclusive(startIndex, endIndex, list), subListExclusive.size());

         for (int i = startIndex + 1; i < endIndex; i++)
            assertEquals(getWrap(i, list), subListExclusive.get(i - startIndex - 1));
      }

      for (int index = 0; index < listSize; index++)
      {
         int startIndex = index;
         int endIndex = index + listSize / 2 - 1;
         List<Object> subListExclusive = subListExclusive(startIndex, endIndex, list);
         assertEquals(subLengthExclusive(startIndex, endIndex, list), subListExclusive.size());

         for (int i = startIndex + 1; i < endIndex; i++)
            assertEquals(getWrap(i, list), subListExclusive.get(i - startIndex - 1));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration =  0.0)
   @Test(timeout = 10000)
   public void testRemoveAllInclusive()
   {
      Random random = new Random(453L);
      List<Object> list;
      List<Object> listBackup;
      list = createList(9, random);
      listBackup = new ArrayList<>(list);
      int from = 8;
      int to = 1;
      int n = ListWrappingIndexTools.removeAllInclusive(from, to, list);
      assertEquals(3, n);
      assertEquals(6, list.size());
      int index = 0;
      for (int i = to + 1; i < from; i++)
         assertEquals(listBackup.get(i), list.get(index++));

      list = createList(12, random);
      listBackup = new ArrayList<>(list);
      from = 11;
      to = 3;
      n = ListWrappingIndexTools.removeAllInclusive(from, to, list);
      assertEquals(5, n);
      assertEquals(7, list.size());
      index = 0;
      for (int i = to + 1; i < from; i++)
         assertEquals(listBackup.get(i), list.get(index++));

      list = createList(20, random);
      listBackup = new ArrayList<>(list);
      from = 11;
      to = 18;
      n = ListWrappingIndexTools.removeAllInclusive(from, to, list);
      assertEquals(8, n);
      assertEquals(12, list.size());
      index = 0;
      for (int i = to + 1; i < from; i++)
         assertEquals(listBackup.get(i), list.get(index++));
   }

   @ContinuousIntegrationTest(estimatedDuration =  0.0)
   @Test(timeout = 10000)
   public void testRemoveAllExclusive()
   {
      Random random = new Random(453L);
      List<Object> list;
      List<Object> listBackup;
      list = createList(9, random);
      listBackup = new ArrayList<>(list);
      int from = 8;
      int to = 1;
      int n = ListWrappingIndexTools.removeAllExclusive(from, to, list);
      assertEquals(1, n);
      assertEquals(8, list.size());
      int index = 0;
      for (int i = to; i <= from; i++)
         assertEquals(listBackup.get(i), list.get(index++));

      list = createList(12, random);
      listBackup = new ArrayList<>(list);
      from = 11;
      to = 3;
      n = ListWrappingIndexTools.removeAllExclusive(from, to, list);
      assertEquals(3, n);
      assertEquals(9, list.size());
      index = 0;
      for (int i = to; i <= from; i++)
         assertEquals(listBackup.get(i), list.get(index++));

      list = createList(20, random);
      listBackup = new ArrayList<>(list);
      from = 11;
      to = 18;
      n = ListWrappingIndexTools.removeAllExclusive(from, to, list);
      assertEquals(6, n);
      assertEquals(14, list.size());
      index = 0;
      for (int i = to; i <= from; i++)
         assertEquals(listBackup.get(i), list.get(index++));
   }

   private static List<Object> createList(int size, Random random)
   {
      List<Object> ret = new ArrayList<>();
      while (ret.size() < size)
         ret.add(new Object());
      return ret;
   }

   public static void main(String[] args) throws URISyntaxException, IOException
   {
      String targetTests = "us.ihmc.robotEnvironmentAwareness.geometry.ListWrappingIndexToolsTest";
      String targetClasses = "us.ihmc.robotEnvironmentAwareness.geometry.ListWrappingIndexTools";
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClasses);
   }
}
