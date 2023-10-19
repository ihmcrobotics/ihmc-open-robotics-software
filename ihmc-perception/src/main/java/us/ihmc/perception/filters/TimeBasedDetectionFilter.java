package us.ihmc.perception.filters;

import java.util.LinkedList;
import java.util.Queue;

/**
 * Keep track of how many detections have been had in some amount of time.
 * For example, perhaps you want to know if 5 detections have been had in the last 1 second.
 */
public class TimeBasedDetectionFilter
{
   private final double expiry;
   private final long detectionsThreshold;
   private final Queue<Long> detectionTimes = new LinkedList<>();

   public TimeBasedDetectionFilter(double expiry, long detectionsThreshold)
   {
      this.expiry = expiry;
      this.detectionsThreshold = detectionsThreshold;
   }

   public void registerDetection()
   {
      detectionTimes.offer(System.currentTimeMillis());
   }

   public void update()
   {
      if (!detectionTimes.isEmpty())
      {
         long detectionTime = detectionTimes.peek();

         // If the detection is older than the expiry time, remove from the queue
         if (System.currentTimeMillis() - detectionTime > (expiry * 1000))
         {
            detectionTimes.poll();
         }
      }
   }

   public boolean isDetected()
   {
      return detectionTimes.size() >= detectionsThreshold;
   }
}
