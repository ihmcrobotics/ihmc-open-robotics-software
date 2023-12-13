package us.ihmc.perception.filters;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class TimeBasedDetectionFilterTest
{
   @Test
   public void timeBasedDetectionFilterPlentyOfDetections()
   {
      // Within the last 0.1 seconds, we need 50 detections
      TimeBasedDetectionFilter timeBasedDetectionFilter = new TimeBasedDetectionFilter(0.1, 50);

      while (!timeBasedDetectionFilter.isDetected())
      {
         timeBasedDetectionFilter.registerDetection();
      }

      assertTrue(timeBasedDetectionFilter.isDetected());
   }

   @Test
   public void timeBasedDetectionFilterNotEnoughDetections() throws InterruptedException
   {
      // Within the last 0.1 seconds, we need 50 detections
      TimeBasedDetectionFilter timeBasedDetectionFilter = new TimeBasedDetectionFilter(0.1, 50);

      while (!timeBasedDetectionFilter.isDetected())
      {
         timeBasedDetectionFilter.registerDetection();
      }

      // Add some time delay (longer than the timeWindow above) so the update() removes elements from the queue
      Thread.sleep(200);

      assertFalse(timeBasedDetectionFilter.isDetected());
   }
}
