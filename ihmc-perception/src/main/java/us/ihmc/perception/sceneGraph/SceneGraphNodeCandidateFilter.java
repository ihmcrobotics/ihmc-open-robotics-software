package us.ihmc.perception.sceneGraph;

import gnu.trove.iterator.TFloatIterator;
import gnu.trove.list.TFloatList;
import gnu.trove.list.linked.TFloatLinkedList;
import us.ihmc.commons.thread.Notification;

public class SceneGraphNodeCandidateFilter
{
   public final int HISTORY = 30; // 1 second at 30 HZ
   public final float ACCEPTANCE_THRESHOLD = 0.6f;

   private final TFloatList detections = new TFloatLinkedList();
   private final Notification detected = new Notification();
   private boolean isStableDetectionResult;

   public void registerDetection()
   {
      detected.set();
   }

   public boolean isStableDetectionResult()
   {
      return isStableDetectionResult;
   }

   public void update()
   {
      detections.add(detected.poll() ? 1.0f : 0.0f);

      while (detections.size() > HISTORY)
         detections.removeAt(0);

      float average = 0.0f;
      for (TFloatIterator iterator = detections.iterator(); iterator.hasNext(); )
      {
         average += iterator.next();
      }
      average /= (float) detections.size();

      isStableDetectionResult = detections.size() == HISTORY && average >= ACCEPTANCE_THRESHOLD;
   }
}
