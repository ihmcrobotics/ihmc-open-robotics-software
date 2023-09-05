package us.ihmc.perception.sceneGraph.rigidBodies;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.PredefinedRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.SceneNode;

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
   private final DetectableSceneNode parentNode;
   /**
    * We don't want to lock in the static pose until we are close enough
    * for it to matter and also to get higher accuracy.
    */
   private double distanceToDisableTracking;
   private double currentDistance = Double.NaN;

   public StaticRelativeSceneNode(String name,
                                  DetectableSceneNode parentNode,
                                  RigidBodyTransform transformToParentNode,
                                  String visualModelFilePath,
                                  RigidBodyTransform visualModelToNodeFrameTransform,
                                  double distanceToDisableTracking)
   {
      super(name, visualModelFilePath, visualModelToNodeFrameTransform);
      this.parentNode = parentNode;

      parentNode.getChildren().add(this);
      setParentFrame(parentNode::getNodeFrame);
      changeParentFrame(parentNode.getNodeFrame());
      getNodeToParentFrameTransform().set(transformToParentNode);
      getNodeFrame().update();

      this.distanceToDisableTracking = distanceToDisableTracking;
   }

   @Override
   public boolean getCurrentlyDetected()
   {
      return !getTrackDetectedPose() || parentNode.getCurrentlyDetected();
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

   public SceneNode getParentNode()
   {
      return parentNode;
   }
}
