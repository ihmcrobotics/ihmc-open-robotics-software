package us.ihmc.perception.sceneGraph.rigidBodies;

import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * This node stays in the same spot relative to where a parent scene node
 * at the time it is seen up close.
 *
 * Once it has been seen up close, the pose of this node is set as known
 * and does not move until {@link #setTrackDetectedPose} is called.
 *
 * The whole point of this is so we don't have to put markers on everything,
 * especially things that don't move.
 */
public class StaticRelativeSceneNode extends PredefinedRigidBodySceneNode
{
   /**
    * We don't want to lock in the static pose until we are close enough
    * for it to matter and also to get higher accuracy.
    */
   private double distanceToDisableTracking;
   private double currentDistance = Double.NaN;

   public StaticRelativeSceneNode(long id,
                                  String name,
                                  String visualModelFilePath,
                                  RigidBodyTransform visualModelToNodeFrameTransform,
                                  double distanceToDisableTracking)
   {
      super(id, name, visualModelFilePath, visualModelToNodeFrameTransform);

      this.distanceToDisableTracking = distanceToDisableTracking;
   }

   public void setDistanceToDisableTracking(double distanceToDisableTracking)
   {
      this.distanceToDisableTracking = distanceToDisableTracking;
   }

   public double getDistanceToDisableTracking()
   {
      return distanceToDisableTracking;
   }

   public void setCurrentDistance(double currentDistance)
   {
      this.currentDistance = currentDistance;
   }

   public double getCurrentDistance()
   {
      return currentDistance;
   }
}
