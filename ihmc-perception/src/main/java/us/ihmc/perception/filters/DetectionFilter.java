package us.ihmc.perception.filters;

import gnu.trove.iterator.TFloatIterator;
import gnu.trove.list.TFloatList;
import gnu.trove.list.linked.TFloatLinkedList;
import us.ihmc.commons.thread.Notification;
import us.ihmc.perception.sceneGraph.SceneGraph;

/**
 * This is for filtering the acceptance of detected objects in
 * the world before considering them to exist, to exclude flickery
 * false positives.
 */
public class DetectionFilter
{
   public final int HISTORY_LENGTH = (int) SceneGraph.UPDATE_FREQUENCY; // 1 second at scene graph's frequency
   public float acceptanceThreshold = 0.6f;

   private final TFloatList detections = new TFloatLinkedList();
   private final Notification detected = new Notification();
   private boolean isStableDetectionResult = false;

   public DetectionFilter()
   {
      // use default threshold
   }

   public DetectionFilter(float acceptanceThreshold)
   {
      this.acceptanceThreshold = acceptanceThreshold;
   }

   /**
    * Lets the filter know that an object has been detected.
    * Should be called in the SceneGraph's update loop.
    */
   public void registerDetection()
   {
      detected.set();
   }

   public boolean isStableDetectionResult()
   {
      return isStableDetectionResult;
   }

   public boolean hasEnoughSamples()
   {
      return detections.size() >= HISTORY_LENGTH;
   }

   public void setAcceptanceThreshold(float threshold)
   {
      this.acceptanceThreshold = threshold;
   }

   public void update()
   {
      detections.add(detected.poll() ? 1.0f : 0.0f);

      while (detections.size() > HISTORY_LENGTH)
         detections.removeAt(0);

      float average = 0.0f;
      for (TFloatIterator iterator = detections.iterator(); iterator.hasNext(); )
      {
         average += iterator.next();
      }
      average /= (float) detections.size();

      isStableDetectionResult = detections.size() == HISTORY_LENGTH && average >= acceptanceThreshold;
   }
}
