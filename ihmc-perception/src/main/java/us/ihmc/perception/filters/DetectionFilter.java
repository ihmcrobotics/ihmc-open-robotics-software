package us.ihmc.perception.filters;

import gnu.trove.iterator.TFloatIterator;
import gnu.trove.list.TFloatList;
import gnu.trove.list.linked.TFloatLinkedList;
import us.ihmc.commons.thread.Notification;

/**
 * This is for filtering the acceptance of detected objects in
 * the world before considering them to exist, to exclude flickery
 * false positives.
 */
public class DetectionFilter
{
   public int historyLength = 30; // 1 second at 30 HZ
   public float acceptanceThreshold = 0.6f;
   public float detectionThreshold = 0.2f;

   private final TFloatList detections = new TFloatLinkedList();
   private final Notification detected = new Notification();
   private boolean isAcceptable = false;
   private boolean isDetected = false;

   public DetectionFilter()
   {
      // use default values
   }

   public DetectionFilter(int historyLength, float acceptanceThreshold, float detectionThreshold)
   {
      this.historyLength = historyLength;
      this.acceptanceThreshold = acceptanceThreshold;
      this.detectionThreshold = detectionThreshold;
   }

   public void registerDetection()
   {
      detected.set();
   }

   public boolean isAcceptable()
   {
      return isAcceptable;
   }

   public boolean isDetected()
   {
      return isDetected;
   }

   public boolean hasEnoughSamples()
   {
      return detections.size() >= historyLength;
   }

   public void update()
   {
      detections.add(detected.poll() ? 1.0f : 0.0f);

      while (detections.size() > historyLength)
         detections.removeAt(0);

      float average = 0.0f;
      for (TFloatIterator iterator = detections.iterator(); iterator.hasNext(); )
      {
         average += iterator.next();
      }
      average /= (float) detections.size();

      isAcceptable = detections.size() == historyLength && average >= acceptanceThreshold;
      isDetected = detections.size() == historyLength && average >= detectionThreshold;
   }
}
