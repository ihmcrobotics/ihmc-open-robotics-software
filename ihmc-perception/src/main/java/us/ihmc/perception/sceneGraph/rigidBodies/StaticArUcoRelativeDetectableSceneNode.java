package us.ihmc.perception.sceneGraph.rigidBodies;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.arUco.ArUcoDetectableNode;

/**
 * This node stays in the same spot relative to a detected ArUco marker.
 *
 * Once the ArUco marker is seen, the pose of this node is set as known
 * and does not move until {@link #unlockPose} is called.
 *
 * The whole point of this is so we don't have to put markers on everything,
 * especially things that don't move.
 */
public class StaticArUcoRelativeDetectableSceneNode extends ArUcoDetectableNode
{
   private boolean poseIsLockedIn = false;
   /**
    * We don't want to lock in the static pose until we are close enough
    * for it to matter and also to get higher accuracy.
    */
   private final double maximumDistanceToLockIn;

   public StaticArUcoRelativeDetectableSceneNode(String name,
                                                 int markerID,
                                                 double markerSize,
                                                 RigidBodyTransform markerTransformToParent,
                                                 String visualModelFilePath,
                                                 RigidBodyTransform visualModelToNodeFrameTransform,
                                                 double maximumDistanceToLockIn)
   {
      super(name, markerID, markerSize, markerTransformToParent, visualModelFilePath, visualModelToNodeFrameTransform);
      this.maximumDistanceToLockIn = maximumDistanceToLockIn;
   }

   public void setPoseIsLockedIn(boolean poseIsLockedIn)
   {
      if (poseIsLockedIn)
         lockInPose();
      else
         unlockPose();
   }

   public void lockInPose()
   {
      poseIsLockedIn = true;
      setCurrentlyDetected(true);
   }

   public void unlockPose()
   {
      setCurrentlyDetected(false);
      poseIsLockedIn = false;
   }

   public double getMaximumDistanceToLockIn()
   {
      return maximumDistanceToLockIn;
   }

   public boolean getPoseIsLockedIn()
   {
      return poseIsLockedIn;
   }
}
