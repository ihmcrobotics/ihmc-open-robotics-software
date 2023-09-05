package us.ihmc.footstepPlanning.tools;

import org.junit.jupiter.api.Test;
import org.ojalgo.random.RandomNumber;
import us.ihmc.commons.RandomNumbers;

import java.util.PriorityQueue;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class LinkedPriorityQueueTest
{
   @Test
   public void testOrderedDoubles()
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

      queue.clear();

      for (double x = 1.0; x <= 10.0; x += 1.0)
      {
         queue.add(x);
      }

      size = 10;
      assertEquals(size, queue.size());

      for(double expected = 1.0; expected <= 10.0; expected += 1.0)
      {
         assertEquals(size--, queue.size());
         assertEquals(expected, queue.pollFirst(), 1e-5);
         assertEquals(size, queue.size());
      }
   }

   @Test
   public void testRandomDoubles()
   {
      LinkedPriorityQueue<Double> linkedQueue = new LinkedPriorityQueue<>(Double::compareTo);
      PriorityQueue<Double> queue = new PriorityQueue<>(Double::compareTo);

      Random random = new Random(1738L);
      for (int i = 0; i < 1000; i++)
      {
         int points = RandomNumbers.nextInt(random, 10, 1000);
         queue.clear();
         linkedQueue.clear();
         for (int j = 0; j < points; j++)
         {
            double num = RandomNumbers.nextDouble(random, 1000);
            queue.add(num);
            linkedQueue.add(num);
         }

         assertEquals(queue.size(), linkedQueue.size());
         for (int j = 0; j < points; j++)
         {
            String message = "Failed iter " + i + " on number " + j;
            assertEquals(message, queue.poll(), linkedQueue.pollFirst(), 1e-6);
         }
      }
   }
}
