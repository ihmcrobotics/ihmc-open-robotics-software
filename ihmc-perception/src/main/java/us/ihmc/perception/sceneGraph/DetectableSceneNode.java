package us.ihmc.perception.sceneGraph;

/**
 * An object that is currently detected or not currently detected,
 * as such with objects tracked via ArUco markers or YOLO.
 */
public class DetectableSceneNode extends SceneNode
{
   private boolean currentlyDetected;

   public DetectableSceneNode(long id, String name)
   {
      super(id, name);
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
