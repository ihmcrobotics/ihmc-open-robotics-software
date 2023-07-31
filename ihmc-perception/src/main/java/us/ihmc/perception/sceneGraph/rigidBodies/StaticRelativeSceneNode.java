package us.ihmc.perception.sceneGraph.rigidBodies;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.PredefinedRigidBodySceneNode;

/**
 * This node stays in the same spot relative to where a parent scene node
 * at the time it is seen up close.
 *
 * Once it has been seen up close, the pose of this node is set as known
 * and does not move until {@link #invalidatePose} is called.
 *
 * The whole point of this is so we don't have to put markers on everything,
 * especially things that don't move.
 */
public class StaticRelativeSceneNode extends PredefinedRigidBodySceneNode
{
   private final RigidBodyTransform originalTransformToParentNode;
   private boolean poseIsStatic = false;
   private final DetectableSceneNode parentNode;
   /**
    * We don't want to lock in the static pose until we are close enough
    * for it to matter and also to get higher accuracy.
    */
   private final double maximumDistanceToLockIn;

   public StaticRelativeSceneNode(String name,
                                  DetectableSceneNode parentNode,
                                  RigidBodyTransform transformToParentNode,
                                  String visualModelFilePath,
                                  RigidBodyTransform visualModelToNodeFrameTransform,
                                  double maximumDistanceToLockIn)
   {
      super(name, visualModelFilePath, visualModelToNodeFrameTransform);
      this.parentNode = parentNode;
      this.originalTransformToParentNode = transformToParentNode;

      changeParentFrame(parentNode.getNodeFrame());
      getNodeToParentFrameTransform().set(transformToParentNode);
      getNodeFrame().update();

      this.maximumDistanceToLockIn = maximumDistanceToLockIn;
   }

   public void setPoseToStatic()
   {
      setPoseIsStatic(true);
   }

   public void invalidatePose()
   {
      setPoseIsStatic(false);
   }

   public void setPoseIsStatic(boolean poseIsStatic)
   {
      boolean changed = this.poseIsStatic != poseIsStatic;
      this.poseIsStatic = poseIsStatic;

      if (changed)
      {
         if (poseIsStatic)
         {
            changeParentFrameWithoutMoving(ReferenceFrame.getWorldFrame());
         }
         else if (getPoseOverriddenByOperator())
         {
            changeParentFrameWithoutMoving(parentNode.getNodeFrame());
         }
         else
         {
            changeParentFrame(parentNode.getNodeFrame());
            getNodeToParentFrameTransform().set(originalTransformToParentNode);
            getNodeFrame().update();
         }
      }
   }

   public double getMaximumDistanceToLockIn()
   {
      return maximumDistanceToLockIn;
   }

   public boolean getPoseIsStatic()
   {
      return poseIsStatic;
   }

   /** @deprecated Doesn't apply to this node, as it's detected via it's parent. */
   @Override
   public void setCurrentlyDetected(boolean currentlyDetected)
   {

   }

   /** This node is not detected directly. */
   @Override
   public boolean getCurrentlyDetected()
   {
      return poseIsStatic || parentNode.getCurrentlyDetected();
   }

   public DetectableSceneNode getParentNode()
   {
      return parentNode;
   }
}
