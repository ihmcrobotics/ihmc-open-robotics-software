package us.ihmc.sensors.zed;

import us.ihmc.euclid.transform.RigidBodyTransform;

import java.time.Instant;
import java.util.ArrayDeque;
import java.util.Deque;

public class TransformBuffer
{
   private final Deque<TimestampedTransform> buffer = new ArrayDeque<>();
   private final int maxSize;

   public TransformBuffer(int maxSize)
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
      TimestampedTransform best = null;

      for (TimestampedTransform t : buffer)
      {
         if (t.timestamp <= time)
            best = t;
         else
            break;
      }

      return best != null ? new RigidBodyTransform(best.transform) : null;
   }


   public class TimestampedTransform
   {
      public final long timestamp;
      public final RigidBodyTransform transform;

      public TimestampedTransform(long timestamp, RigidBodyTransform transform)
      {
         this.timestamp = timestamp;
         this.transform = transform;
      }
   }
}