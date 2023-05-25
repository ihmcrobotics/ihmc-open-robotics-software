package us.ihmc.perception.sceneGraph;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.tools.Timer;

/**
 * An object that is currently detected or not currently detected,
 * as such with objects tracked via ArUco markers or YOLO.
 */
public abstract class DetectableSceneNode extends SceneNode
{
   /**
    * Scene nodes are usually being synced at 20 Hz or faster, so 1/5 of a second
    * should allow enough time for changes to propagate.
    */
   public static final double OPERATOR_FREEZE_TIME = 0.2;

   private boolean currentlyDetected;
   /**
    * We allow the operator to override the pose of a detectable scene node.
    * This keeps the node in the reference frame tree but keeps the node pinned
    * with respect to world frame.
    */
   private boolean isPoseOverriddenByOperator = false;
   private final FramePose3D storedOverriddenPose = new FramePose3D();
   /**
    * This timer is used in the case that an operator can "mark modified" this node's
    * data so it won't accept updates from other sources for a short period of time.
    * This is to allow the changes to propagate elsewhere.
    */
   private final Timer modifiedTimer = new Timer();

   public DetectableSceneNode(String name)
   {
      super(name);
   }

   public DetectableSceneNode(String name, ReferenceFrame parentFrame)
   {
      super(name, parentFrame);
   }

   public void setCurrentlyDetected(boolean currentlyDetected)
   {
      this.currentlyDetected = currentlyDetected;
   }

   public boolean getCurrentlyDetected()
   {
      return currentlyDetected;
   }

   public boolean getPoseOverriddenByOperator()
   {
      return isPoseOverriddenByOperator;
   }

   public void setPoseOverriddenByOperator(boolean poseOverriddenByOperator)
   {
      this.isPoseOverriddenByOperator = poseOverriddenByOperator;
   }

   public void storeOverriddenPose()
   {
      storedOverriddenPose.setIncludingFrame(getNodeFrame().getParent(), getNodeToParentFrameTransform());
      storedOverriddenPose.changeFrame(ReferenceFrame.getWorldFrame()); // We need to store it in world frame
   }

   public void restoreOverriddenPose()
   {
      // At this point the node frame parent has presumably moved, so let's go back to that frame.
      storedOverriddenPose.changeFrame(getNodeFrame().getParent());
      storedOverriddenPose.get(getNodeToParentFrameTransform());
      getNodeFrame().update();
   }

   public void markModifiedByOperator()
   {
      modifiedTimer.reset();
   }

   public boolean noLongerFrozenByOperator()
   {
      return !modifiedTimer.isRunning(OPERATOR_FREEZE_TIME);
   }
}
