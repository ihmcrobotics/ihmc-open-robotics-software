package us.ihmc.perception.sceneGraph;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.tools.Timer;

import javax.annotation.Nullable;

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
   public static final double OPERATOR_FREEZE_TIME = 1.0;
   /**
    * This timer is used in the case that an operator can "mark modified" this node's
    * data so it won't accept updates from other sources for a short period of time.
    * This is to allow the changes to propagate elsewhere.
    */
   private final Timer modifiedTimer = new Timer();

   private boolean currentlyDetected;
   /**
    * We allow the operator to disable tracking the detected pose.
    */
   private boolean trackDetectedPose = true;
   @Nullable
   private SceneNode parentNode;
   private final RigidBodyTransform originalTransformToParent = new RigidBodyTransform();
   private transient final FramePose3D originalPose = new FramePose3D();

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

   public boolean getTrackDetectedPose()
   {
      return trackDetectedPose;
   }

   public void setOriginalTransformToParent(RigidBodyTransform originalTransformToParent)
   {
      this.originalTransformToParent.set(originalTransformToParent);
   }

   /**
    * Sets whether this node is tracking the detected pose.
    * This works by detaching or reattaching this node to/from it's parent
    * reference frame. Warning: ReferenceFrames have immutable parent reference,
    * so the reference frame representing this node will get recreated.
    */
   public void setTrackDetectedPose(boolean trackDetectedPose)
   {
      this.trackDetectedPose = trackDetectedPose;

      if (parentNode != null)
      {
         if (trackDetectedPose && parentNode.getNodeFrame() != getNodeFrame().getParent())
         {
            changeParentFrameWithoutMoving(parentNode.getNodeFrame());
         }
         else if (!trackDetectedPose && getNodeFrame().getParent() != ReferenceFrame.getWorldFrame())
         {
            changeParentFrameWithoutMoving(ReferenceFrame.getWorldFrame());
         }
      }
   }

   /**
    * This sets the transform to the parent node back to the original one.
    * This is robust to whether or not this node is currently tracking the detected pose.
    */
   public void clearOffset()
   {
      if (parentNode != null && parentNode.getNodeFrame() != getNodeFrame().getParent())
      {
         originalPose.setToZero(parentNode.getNodeFrame());
         originalPose.set(originalTransformToParent);
         originalPose.changeFrame(getNodeFrame().getParent());
         originalPose.get(getNodeToParentFrameTransform());
      }
      else
      {
         getNodeToParentFrameTransform().set(originalTransformToParent);
      }
      getNodeFrame().update();
   }

   public void markModifiedByOperator()
   {
      modifiedTimer.reset();
   }

   public boolean operatorHasntModifiedThisRecently()
   {
      return !modifiedTimer.isRunning(OPERATOR_FREEZE_TIME);
   }

   public void setParentNode(SceneNode sceneNode)
   {
      parentNode = sceneNode;
   }

   @Nullable
   public SceneNode getParentNode()
   {
      return parentNode;
   }
}
