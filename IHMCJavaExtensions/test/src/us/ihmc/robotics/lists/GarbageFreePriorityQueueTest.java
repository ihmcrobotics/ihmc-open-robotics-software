package us.ihmc.robotics.lists;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;

public class GarbageFreePriorityQueueTest
{

   @Test
   public void testQueueablepriorityQueue()
   {
      PriorityQueue<TestComparable> priorityQueue = new PriorityQueue<TestComparable>(10, TestComparable.class);
      for(int i = 0; i < 10; i++)
      {
         TestComparable comparable = new TestComparable();
         comparable.setData(i);
         assertTrue(priorityQueue.add(comparable));
      }

      for(int i = 0; i < 10; i++)
      {
         TestComparable comparable = new TestComparable();
         assertFalse(priorityQueue.add(comparable));
      }
   }

   @Test
   public void testSameDelay()
   {
      PriorityQueue<TestComparable> priorityQueue = new PriorityQueue<TestComparable>(10, TestComparable.class);
      for(int i = 0; i < 10; i++)
      {
         TestComparable comparable = new TestComparable();
         comparable.setData(5.0);
         assertTrue(priorityQueue.add(comparable));
      }
      
      int index = 0;
      while(priorityQueue.peek() != null)
      {
         TestComparable comparable = priorityQueue.pop();
         assertEquals(9 - index, priorityQueue.getSize());
         assertEquals(5.0, comparable.getData(), 1e-10);
         index++;
      }
      assertEquals(10, index);
   }

   @Test
   public void testAddingInOrder()
   {
      PriorityQueue<TestComparable> priorityQueue = new PriorityQueue<TestComparable>(10, TestComparable.class);
      for(int i = 0; i < 10; i++)
      {
         TestComparable comparable = new TestComparable();
         comparable.setData(i);
         assertTrue(priorityQueue.add(comparable));
      }
      
      int index = 0;
      while(priorityQueue.peek() != null)
      {
         TestComparable comparable = priorityQueue.pop();
         assertEquals(index, comparable.getData(), 1e-10);
         index++;
      }
      assertEquals(10, index);
   }

   @Test
   public void testPop()
   {
      PriorityQueue<TestComparable> priorityQueue = new PriorityQueue<TestComparable>(10, TestComparable.class);
      TestComparable[] comparables = new TestComparable[10];
      for(int i = 0; i < 10; i++)
      {
         TestComparable comparable = new TestComparable();
         comparable.setData(10 - i);
         comparables[9 - i] = comparable;
         assertTrue(priorityQueue.add(comparable));
      }
      
      int index = 0;
      while(priorityQueue.peek() != null)
      {
         assertEquals(comparables[index], priorityQueue.pop());
         index++;
      }
   }

   @Test
   public void testPopExtended()
   {
      int numberOfCommands = 100;
      ArrayList<TestComparable> comparablesInRandomOrder = new ArrayList<>();
      Random random = new Random(100);
      PriorityQueue<TestComparable> priorityQueue = new PriorityQueue<TestComparable>(numberOfCommands, TestComparable.class);
      TestComparable[] comparables = new TestComparable[numberOfCommands];
      
      //get random numbers
      double[] delays = new double[numberOfCommands];
      for(int i = 0; i < delays.length; i++)
      {
         delays[i] = random.nextDouble() * random.nextInt(1000);
      }
      
      //sort them
      Arrays.sort(delays);
      
      for(int i = 0; i < delays.length; i++)
      {
         TestComparable comparable = new TestComparable();
         comparable.setData(delays[i]);
         comparables[i] = comparable;
         comparablesInRandomOrder.add(random.nextInt(comparablesInRandomOrder.size() + 1), comparable);
      }
      
      //put the commands in the priority queue
      for(int i = 0; i < comparablesInRandomOrder.size(); i++)
      {
         TestComparable comparable = comparablesInRandomOrder.get(i);
         assertTrue(priorityQueue.add(comparable));
      }
      
      //test that the output matches the sorted queue
      int index = 0;
      while(priorityQueue.peek() != null)
      {
         assertEquals(comparables[index], priorityQueue.pop());
         index++;
      }
   }
   
   @Test
   public void testPeek()
   { 
      PriorityQueue<TestComparable> priorityQueue = new PriorityQueue<TestComparable>(10, TestComparable.class);
      TestComparable comparable = new TestComparable();
      comparable.setData(5.0);
      assertTrue(priorityQueue.add(comparable));
      assertEquals(comparable, priorityQueue.peek());
      assertEquals(1, priorityQueue.getSize());
   }
   
   @Test
   public void testPopWhenEmpty()
   { 
      PriorityQueue<TestComparable> priorityQueue = new PriorityQueue<TestComparable>(10, TestComparable.class);
      assertNull(priorityQueue.pop());
      assertEquals(0, priorityQueue.getSize());
   }

   @Test
   public void testClear()
   { 
      PriorityQueue<TestComparable> priorityQueue = new PriorityQueue<TestComparable>(10, TestComparable.class);
      for(int i = 0; i < 10; i++)
      {
         TestComparable comparable = new TestComparable();
         comparable.setData(i);
         assertTrue(priorityQueue.add(comparable));
      }
      
      priorityQueue.clear();
      int index = 0;
      while(priorityQueue.peek() != null)
      {
         TestComparable comparable = priorityQueue.pop();
         assertEquals(index, comparable.getData(), 1e-10);
         index++;
      }
      assertEquals(0, index);
      assertEquals(0, priorityQueue.getSize());
      priorityQueue.clear();
      assertEquals(0, index);
      assertEquals(0, priorityQueue.getSize());
   }
   
   private class TestComparable implements Comparable<TestComparable>
   {
      private double data;
      
      public double getData()
      {
         return data;
      }
      
      public void setData(double comparable)
      {
         this.data = comparable;
      }
      
      @Override
      public int compareTo(TestComparable other)
      {
         double diff = this.data - other.data;
         
         if(diff < 0)
         {
            return -1;
         }
         
         if(diff > 0)
         {
            return 1;
         }
         
         return 0;
      }
      
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(PriorityQueue.class, GarbageFreePriorityQueueTest.class);
   }
}
