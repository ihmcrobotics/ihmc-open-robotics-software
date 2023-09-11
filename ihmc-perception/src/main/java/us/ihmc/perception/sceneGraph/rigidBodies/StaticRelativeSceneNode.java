package us.ihmc.perception.sceneGraph.rigidBodies;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.perception.sceneGraph.SceneNode;

import java.util.function.Supplier;

/**
 * This node stays in the same spot relative to where a parent scene node
 * at the time it is seen up close.
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
                                  Supplier<SceneNode> initialParentNodeSupplier,
                                  RigidBodyTransformReadOnly initialTransformToParent,
                                  String visualModelFilePath,
                                  RigidBodyTransform visualModelToNodeFrameTransform,
                                  double distanceToDisableTracking)
   {
      super(id, name, initialParentNodeSupplier, initialTransformToParent, visualModelFilePath, visualModelToNodeFrameTransform);

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
