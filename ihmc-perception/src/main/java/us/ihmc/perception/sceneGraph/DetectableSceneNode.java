package us.ihmc.perception.sceneGraph;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

/**
 * An object that is currently detected or not currently detected,
 * as such with objects tracked via ArUco markers or YOLO.
 */
public abstract class DetectableSceneNode extends SceneNode
{
   private boolean currentlyDetected;
   /**
    * We allow the operator to override the pose of a detectable scene node.
    * This keeps the node in the reference frame tree but keeps the node pinned
    * with respect to world frame.
    */
   private boolean isPoseOverriddenByOperator = false;
   private final FramePose3D storedOverriddenPose = new FramePose3D();

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
      storedOverriddenPose.setIncludingFrame(getNodeFrame(), getNodeToParentFrameTransform());
   }

   public void restoreOverriddenPose()
   {
      storedOverriddenPose.changeFrame(getNodeFrame().getParent());
      storedOverriddenPose.get(getNodeFrame().getTransformToWorldFrame());
      getNodeFrame().update();
   }
}
