package us.ihmc.communication.crdt;

import java.util.PriorityQueue;

public class LimitedSortedLongSet extends PriorityQueue<Long>
{
   private final int maxSize;

   public LimitedSortedLongSet(int maxSize)
   {
      this.maxSize = maxSize;
   }

   public void addWithLimit(long element)
   {
      offer(element);

      while (size() > maxSize)
         poll();
   }
}
