package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import static org.junit.Assert.assertTrue;

import java.lang.reflect.Array;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.RecycledLinkedListBuilder.RecycledLinkedListEntry;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.lists.GenericTypeBuilder;

public class RecycledLinkedListBuilderTest
{
   private static int num = 0;

   public class DummyClass
   {
      int number;

      public DummyClass()
      {
         number = num++;
      }
   }

   @Test
   public void testConstructor()
   {
      RecycledLinkedListBuilder<DummyClass> linkedList = new RecycledLinkedListBuilder<>(createBuilderForDummyClass());
      assertTrue(linkedList.getSize() == 0);
   }

   @Test
   public void testWithTwoElements()
   {
      num = 0;
      RecycledLinkedListBuilder<DummyClass> linkedList = new RecycledLinkedListBuilder<>(createBuilderForDummyClass());
      assertTrue(linkedList.getSize() == 0);
      assertTrue(linkedList.getFirstEntry() == null);
      assertTrue(linkedList.getLastEntry() == null);
      RecycledLinkedListBuilder<DummyClass>.RecycledLinkedListEntry<DummyClass> firstEntry = linkedList.getOrCreateFirstEntry();
      assertTrue(firstEntry != null);
      assertTrue(firstEntry.getNext() == null);
      assertTrue(firstEntry.getPrevious() == null);
      assertTrue(firstEntry.element.number == (num - 1));
      RecycledLinkedListBuilder<DummyClass>.RecycledLinkedListEntry<DummyClass> secondEntry = linkedList.insertBefore(firstEntry);
      assertTrue(linkedList.getSize() == 2);
      assertTrue(secondEntry.element.number + "", secondEntry.element.number == (num - 1));
      assertTrue(firstEntry.getPrevious() == secondEntry);
      assertTrue(firstEntry.getNext() == null);
      assertTrue(secondEntry.getNext() == firstEntry);
      assertTrue(secondEntry.getPrevious() == null);
      assertTrue(firstEntry != secondEntry);
      assertTrue(secondEntry == linkedList.getFirstEntry());
      assertTrue(firstEntry == linkedList.getLastEntry());
      linkedList.remove(firstEntry);
      assertTrue(linkedList.getSize() == 1);
      assertTrue(linkedList.getLastEntry() == secondEntry);
      assertTrue(secondEntry.getNext() == null);
      assertTrue(secondEntry.getPrevious() == null);
   }

   @Test
   public void testCreatingMultipleElements()
   {
      num = 0;
      RecycledLinkedListBuilder<DummyClass> linkedList = new RecycledLinkedListBuilder<>(createBuilderForDummyClass());
      RecycledLinkedListBuilder<DummyClass>.RecycledLinkedListEntry<DummyClass> firstEntry = linkedList.getOrCreateFirstEntry();
      RecycledLinkedListBuilder<DummyClass>.RecycledLinkedListEntry<DummyClass> secondEntry = linkedList.insertAfter(firstEntry);
      RecycledLinkedListBuilder<DummyClass>.RecycledLinkedListEntry<DummyClass> newSecondEntry = linkedList.insertBefore(secondEntry);
      assertTrue(linkedList.getSize() == 3);
      assertTrue(linkedList.getLastEntry() == secondEntry);
      assertTrue(secondEntry.getPrevious() == newSecondEntry);
      assertTrue(newSecondEntry.getPrevious() == firstEntry);
      assertTrue(firstEntry.getPrevious() == null);
      assertTrue(firstEntry.getNext() == newSecondEntry);
      assertTrue(newSecondEntry.getNext() == secondEntry);
      assertTrue(secondEntry.getNext() == null);
   }

   @Test
   public void testHunderdElementAddition()
   {
      num = 0;
      int numEnteries = 100;
      @SuppressWarnings("unchecked")
      RecycledLinkedListBuilder<DummyClass>.RecycledLinkedListEntry<DummyClass>[] enteriesList = (RecycledLinkedListBuilder<DummyClass>.RecycledLinkedListEntry<DummyClass>[]) Array.newInstance(RecycledLinkedListEntry.class,
                                                                                                                                                                                                 numEnteries);
      RecycledLinkedListBuilder<DummyClass> linkedList = new RecycledLinkedListBuilder<>(numEnteries / 2, createBuilderForDummyClass());
      RecycledLinkedListBuilder<DummyClass>.RecycledLinkedListEntry<DummyClass> currentEntry = linkedList.getOrCreateFirstEntry();
      enteriesList[0] = currentEntry;
      for (int i = 1; i < numEnteries; i++)
      {
         currentEntry = linkedList.insertAfter(currentEntry);
         enteriesList[i] = currentEntry;
      }
      assertTrue(linkedList.getSize() == numEnteries);
      for (int i = 1; i < numEnteries; i++)
      {
         assertTrue(enteriesList[i - 1].getNext() == enteriesList[i]);
         assertTrue(enteriesList[i].getPrevious() == enteriesList[i - 1]);
      }
   }

   @Test
   public void testAdditionRemovalAddition()
   {
      num = 0;
      int numEnteries = 100;
      @SuppressWarnings("unchecked")
      RecycledLinkedListBuilder<DummyClass>.RecycledLinkedListEntry<DummyClass>[] enteriesList = (RecycledLinkedListBuilder<DummyClass>.RecycledLinkedListEntry<DummyClass>[]) Array.newInstance(RecycledLinkedListEntry.class,
                                                                                                                                                                                                 numEnteries);
      RecycledLinkedListBuilder<DummyClass> linkedList = new RecycledLinkedListBuilder<>(numEnteries / 2, createBuilderForDummyClass());
      RecycledLinkedListBuilder<DummyClass>.RecycledLinkedListEntry<DummyClass> currentEntry = linkedList.getOrCreateFirstEntry();
      enteriesList[0] = currentEntry;
      for (int i = 1; i < numEnteries; i++)
      {
         currentEntry = linkedList.insertAfter(currentEntry);
         enteriesList[i] = currentEntry;
      }
      assertTrue(linkedList.getSize() == numEnteries);
      for (int i = 0; i < numEnteries; i++)
         linkedList.remove(enteriesList[i]);
      assertTrue(linkedList.getSize() == 0);

      currentEntry = linkedList.getOrCreateFirstEntry();
      for (int i = 1; i < numEnteries; i++)
      {
         currentEntry = linkedList.insertAfter(currentEntry);
         enteriesList[i] = currentEntry;
         assertTrue(currentEntry.element.number < 100);
      }
   }

   private GenericTypeBuilder<DummyClass> createBuilderForDummyClass()
   {
      return new GenericTypeBuilder<DummyClass>()
      {
         @Override
         public DummyClass newInstance()
         {
            return new DummyClass();
         }
      };
   }
}
