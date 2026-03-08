package us.ihmc.perception;

import us.ihmc.euclid.transform.RigidBodyTransform;

import java.util.ArrayDeque;
import java.util.Deque;

public class BufferedCameraTransform
{
   private final Deque<TimestampedTransform> buffer = new ArrayDeque<>();
   private final int maxSize;

   public BufferedCameraTransform(int maxSize)
   {
      this.maxSize = maxSize;
   }

   public synchronized void add(long time, RigidBodyTransform transform)
   {
      buffer.addLast(new TimestampedTransform(time, new RigidBodyTransform(transform)));

      while (buffer.size() > maxSize)
         buffer.removeFirst();
   }

   public synchronized RigidBodyTransform lookup(long time)
   {
      TimestampedTransform bestTime = null;

      for (TimestampedTransform t : buffer)
      {
         if (t.timestamp <= time)
            bestTime = t;
         else
            break;
      }

      return bestTime != null ? new RigidBodyTransform(bestTime.transform) : null;
   }

   public record TimestampedTransform(long timestamp, RigidBodyTransform transform)
   {
   }
}
