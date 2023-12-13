package us.ihmc.perception.filters;

import java.util.PriorityQueue;

/**
 * Keep track of how many detections have been had in some amount of time.
 * For example, perhaps you want to know if 5 detections have been had in the last 1 second.
 */
public class TimeBasedDetectionFilter
{
   private final double timeWindow;
   private final long requiredNumberOfDetections;
   private final PriorityQueue<Long> detectionTimes = new PriorityQueue<>();

   public TimeBasedDetectionFilter(double timeWindow, long requiredNumberOfDetections)
   {
      this.timeWindow = timeWindow;
      this.requiredNumberOfDetections = requiredNumberOfDetections;
   }

   public void registerDetection()
   {
      detectionTimes.offer(System.currentTimeMillis());
   }

   public boolean isDetected()
   {
      while (!detectionTimes.isEmpty() && (System.currentTimeMillis() - detectionTimes.peek() > (timeWindow * 1000)))
      {
         // If the detection is older than the expiry time, remove from the queue
         detectionTimes.poll();
      }

      return detectionTimes.size() >= requiredNumberOfDetections;
   }
}
