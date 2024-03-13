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
   /**
    * historyLength should be matched to the frequency at which the detection filter is updated.
    * E.g. when used inside the scene graph update loop, historyLength = SceneGraph.UPDATE_FREQUENCY.
    * This will represent a history of 1 second.
    */
   public int historyLength;
   public float acceptanceThreshold;

   private final TFloatList detections = new TFloatLinkedList();
   private final Notification detected = new Notification();
   private boolean isStableDetectionResult = false;

   public DetectionFilter()
   {
      this((int) SceneGraph.UPDATE_FREQUENCY, 0.6f);
   }

   public DetectionFilter(int historyLength, float acceptanceThreshold)
   {
      this.historyLength = historyLength;
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
      return detections.size() >= historyLength;
   }

   public void setHistoryLength(int historyLength)
   {
      this.historyLength = historyLength;
   }

   public void setAcceptanceThreshold(float threshold)
   {
      this.acceptanceThreshold = threshold;
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

      isStableDetectionResult = detections.size() == historyLength && average >= acceptanceThreshold;
   }
}
