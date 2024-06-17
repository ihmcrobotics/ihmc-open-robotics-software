package us.ihmc.perception.sceneGraph;

import us.ihmc.perception.detections.InstantDetection;

/**
 * An object that is currently detected or not currently detected,
 * as such with objects tracked via ArUco markers or YOLO.
 */
public class DetectableSceneNode extends SceneNode
{
   private InstantDetection detection;
   private boolean currentlyDetected;

   public DetectableSceneNode(long id, String name, InstantDetection detection)
   {
      super(id, name);
      this.detection = detection;
   }

   public void updateDetection(InstantDetection newDetection)
   {
      detection = newDetection;
   }

   public InstantDetection getDetection()
   {
      return detection;
   }

   public void setCurrentlyDetected(boolean currentlyDetected)
   {
      this.currentlyDetected = currentlyDetected;
   }

   public boolean getCurrentlyDetected()
   {
      return currentlyDetected;
   }
}
