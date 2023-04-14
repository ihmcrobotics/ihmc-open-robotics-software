package us.ihmc.perception.objects;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.scene.ArUcoDetectableObject;

/**
 * This object stays in the same spot relative to a detected ArUco marker.
 *
 * Once the ArUco marker is seen, the pose of this object is set as known
 * and does not move until {@link #forgetPose} is called.
 */
public class StaticArUcoRelativeDetectableSceneObject extends ArUcoDetectableObject
{
   private boolean poseKnown = false;

   public StaticArUcoRelativeDetectableSceneObject(String name, int markerID, double markerSize, RigidBodyTransform markerTransformToParent)
   {
      super(name, markerID, markerSize, markerTransformToParent);
   }

   public void whate()
   {
      if (!poseKnown)
      {

         poseKnown = true;
      }
   }

   public void forgetPose()
   {
      setCurrentlyDetected(false);
      poseKnown = false;
   }
}
