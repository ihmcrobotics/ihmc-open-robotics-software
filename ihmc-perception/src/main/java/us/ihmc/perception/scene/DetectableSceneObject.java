package us.ihmc.perception.scene;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

/**
 * An object that is currently detected or not currently detected,
 * as such with objects tracked via ArUco markers or YOLO.
 */
public abstract class DetectableSceneObject extends SceneObject
{
   private boolean currentlyDetected;

   public DetectableSceneObject(String name, long uuid)
   {
      super(name, uuid);
   }

   public DetectableSceneObject(String name, long uuid, ReferenceFrame parentFrame)
   {
      super(name, uuid, parentFrame);
   }

   public boolean getCurrentlyDetected()
   {
      return currentlyDetected;
   }
}
