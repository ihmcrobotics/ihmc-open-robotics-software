package us.ihmc.perception.sceneGraph.rigidBodies;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.robotics.referenceFrames.ReferenceFrameSupplier;

/**
 * A scene object that is a rigid body whose shape and appearance is
 * known beforehand.
 *
 * Rigid bodies as in a door panel, chair, can of soup, etc.
 *
 * This class also provides support for remembering the parent frame
 * and original transform to parent, allowing an operator to manually
 * adjust it and also reset it.
 *
 * TODO:
 *   - Add collision information
 */
public class PredefinedRigidBodySceneNode extends SceneNode
{
   private final String visualModelFilePath;
   private final RigidBodyTransform visualModelToNodeFrameTransform;
   /** ID of parent node when tracking. */
   private long originalParentID;
   private ReferenceFrameSupplier originalParentFrameSupplier;
   private final RigidBodyTransform originalTransformToParent = new RigidBodyTransform();
   private transient final FramePose3D originalPose = new FramePose3D();

   public PredefinedRigidBodySceneNode(long id, String name, String visualModelFilePath, RigidBodyTransform visualModelToNodeFrameTransform)
   {
      super(id, name);
      this.visualModelFilePath = visualModelFilePath;
      this.visualModelToNodeFrameTransform = visualModelToNodeFrameTransform;
   }

   public String getVisualModelFilePath()
   {
      return visualModelFilePath;
   }

   public RigidBodyTransform getVisualModelToNodeFrameTransform()
   {
      return visualModelToNodeFrameTransform;
   }

   public void setOriginalParentID(long originalParentID)
   {
      this.originalParentID = originalParentID;
   }

   public long getOriginalParentID()
   {
      return originalParentID;
   }

   public void setOriginalParentFrame(ReferenceFrameSupplier parentFrameSupplier)
   {
      this.originalParentFrameSupplier = parentFrameSupplier;
   }

   public ReferenceFrameSupplier getOriginalParentFrame()
   {
      return originalParentFrameSupplier;
   }

   public void setOriginalTransformToParent(RigidBodyTransform originalTransformToParent)
   {
      this.originalTransformToParent.set(originalTransformToParent);
   }

   /**
    * This sets the transform to the parent node back to the original one.
    * This is robust to whether or not this node is currently tracking the detected pose.
    */
   public void clearOffset()
   {
      if (originalParentFrameSupplier.get() != getNodeFrame().getParent())
      {
         originalPose.setToZero(originalParentFrameSupplier.get());
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

   public RigidBodyTransform getOriginalTransformToParent()
   {
      return originalTransformToParent;
   }
}
