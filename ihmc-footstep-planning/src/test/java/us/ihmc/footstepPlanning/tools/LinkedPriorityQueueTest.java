package us.ihmc.footstepPlanning.tools;

import org.junit.jupiter.api.Test;

import static us.ihmc.robotics.Assert.assertEquals;

public class LinkedPriorityQueueTest
{
   @Test
   public void testDoubles()
   {
      LinkedPriorityQueue<Double> queue = new LinkedPriorityQueue<>(Double::compareTo);

      for (double x = 10.0; x >= 1.0; x -= 1.0)
      {
         queue.add(x);
      }

      int size = 10;
      assertEquals(size, queue.size());

      for(double expected = 1.0; expected <= 10.0; expected += 1.0)
      {
         assertEquals(size--, queue.size());
         assertEquals(expected, queue.pollFirst(), 1e-5);
         assertEquals(size, queue.size());
      }

   }
}
