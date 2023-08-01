package us.ihmc.perception.sceneGraph.rigidBodies;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.PredefinedRigidBodySceneNode;

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
   private final double maximumDistanceToLockIn;
   private final DetectableSceneNode parentNode;

   public StaticRelativeSceneNode(String name,
                                  DetectableSceneNode parentNode,
                                  RigidBodyTransform transformToParentNode,
                                  String visualModelFilePath,
                                  RigidBodyTransform visualModelToNodeFrameTransform,
                                  double maximumDistanceToLockIn)
   {
      super(name, visualModelFilePath, visualModelToNodeFrameTransform);
      this.parentNode = parentNode;

      setParentNode(parentNode);
      changeParentFrame(parentNode.getNodeFrame());
      getNodeToParentFrameTransform().set(transformToParentNode);
      getNodeFrame().update();

      this.maximumDistanceToLockIn = maximumDistanceToLockIn;
   }

   public double getMaximumDistanceToLockIn()
   {
      return maximumDistanceToLockIn;
   }

   @Override
   public boolean getCurrentlyDetected()
   {
      return !getTrackDetectedPose() || parentNode.getCurrentlyDetected();
   }
}
