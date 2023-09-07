package us.ihmc.perception.sceneGraph;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameSupplier;
import us.ihmc.tools.Timer;

import java.util.ArrayList;
import java.util.List;

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
   private ReferenceFrameSupplier parentFrameSupplier;
   private final List<DetectableSceneNode> children = new ArrayList<>();
   private final RigidBodyTransform originalTransformToParent = new RigidBodyTransform();
   private transient final FramePose3D originalPose = new FramePose3D();

   public DetectableSceneNode(long id, String name)
   {
      super(id, name);
   }

   public void setParentFrame(ReferenceFrameSupplier parentFrameSupplier)
   {
      this.parentFrameSupplier = parentFrameSupplier;
   }

   public ReferenceFrameSupplier getParentFrame()
   {
      return parentFrameSupplier;
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

      update();
      for (DetectableSceneNode child : children)
      {
         child.update();
      }
   }

   private void update()
   {
      ReferenceFrame updatedParentFrame = trackDetectedPose ? parentFrameSupplier.get() : ReferenceFrame.getWorldFrame();

      if (EuclidCoreMissingTools.hasBeenRemoved(updatedParentFrame))
      {
         LogTools.error("Parent frame has been removed! Parent name: %s This node: %s",
                        EuclidCoreMissingTools.frameName(updatedParentFrame), getName());
      }

      boolean thisFrameHasBeenRemoved = EuclidCoreMissingTools.hasBeenRemoved(getNodeFrame());

      if (thisFrameHasBeenRemoved || updatedParentFrame != getNodeFrame().getParent())
      {
         changeParentFrameWithoutMoving(updatedParentFrame);
      }
   }

   /**
    * This sets the transform to the parent node back to the original one.
    * This is robust to whether or not this node is currently tracking the detected pose.
    */
   public void clearOffset()
   {
      if (parentFrameSupplier.get() != getNodeFrame().getParent())
      {
         originalPose.setToZero(parentFrameSupplier.get());
         originalPose.set(getOriginalTransformToParent());
         originalPose.changeFrame(getNodeFrame().getParent());
         originalPose.get(getNodeToParentFrameTransform());
      }
      else
      {
         getNodeToParentFrameTransform().set(getOriginalTransformToParent());
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

   public RigidBodyTransform getOriginalTransformToParent()
   {
      return originalTransformToParent;
   }

   public List<DetectableSceneNode> getChildren()
   {
      return children;
   }
}
