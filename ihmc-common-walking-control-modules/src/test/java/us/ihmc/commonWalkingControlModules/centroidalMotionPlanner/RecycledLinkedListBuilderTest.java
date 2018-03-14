package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

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
