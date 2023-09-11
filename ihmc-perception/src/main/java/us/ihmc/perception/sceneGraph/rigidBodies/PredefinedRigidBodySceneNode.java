package us.ihmc.perception.sceneGraph.rigidBodies;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.perception.sceneGraph.SceneNode;

import java.util.function.Supplier;

/**
 * A scene object that is a rigid body whose shape and appearance is
 * known beforehand.
 *
 * Rigid bodies as in a door panel, chair, can of soup, etc.
 *
 * This class also provides support for remembering the parent frame
 * and initial transform to parent, allowing an operator to manually
 * adjust it and also reset it.
 *
 * TODO:
 *   - Add collision information
 */
public class PredefinedRigidBodySceneNode extends SceneNode
{
   private final Supplier<SceneNode> rootNodeSupplier;
   private final Supplier<SceneNode> initialParentNodeSupplier;
   private final RigidBodyTransform initialTransformToParent = new RigidBodyTransform();
   private transient final FramePose3D initialPose = new FramePose3D();
   private final String visualModelFilePath;
   private final RigidBodyTransform visualModelToNodeFrameTransform;

   public PredefinedRigidBodySceneNode(long id,
                                       String name,
                                       Supplier<SceneNode> rootNodeSupplier,
                                       Supplier<SceneNode> initialParentNodeSupplier,
                                       RigidBodyTransformReadOnly initialTransformToParent,
                                       String visualModelFilePath,
                                       RigidBodyTransform visualModelToNodeFrameTransform)
   {
      super(id, name);
      this.rootNodeSupplier = rootNodeSupplier;
      this.initialParentNodeSupplier = initialParentNodeSupplier;
      this.initialTransformToParent.set(initialTransformToParent);
      this.visualModelFilePath = visualModelFilePath;
      this.visualModelToNodeFrameTransform = visualModelToNodeFrameTransform;

      getNodeToParentFrameTransform().set(initialTransformToParent);
      getNodeFrame().update();
   }

   public void setTrackInitialParent(boolean trackInitialParent)
   {
      boolean previousTrackingInitialParent = getNodeFrame().getParent() == initialParentNodeSupplier.get().getNodeFrame();
      if (previousTrackingInitialParent != trackInitialParent)
      {
         if (trackInitialParent)
         {
            rootNodeSupplier.get().getChildren().remove(this);
            initialParentNodeSupplier.get().getChildren().add(this);
         }
         else
         {
            initialParentNodeSupplier.get().getChildren().remove(this);
            rootNodeSupplier.get().getChildren().add(this);
         }
         update(); // Update this and children because parent frame has changed
      }
   }

   /**
    * This sets the transform to the parent node back to the original one.
    * This is robust to whether or not this node is currently tracking the detected pose.
    */
   public void clearOffset()
   {
      if (getNodeFrame().getParent() != initialParentNodeSupplier.get().getNodeFrame())
      {
         SceneNode originalParentNode = initialParentNodeSupplier.get();
         initialPose.setIncludingFrame(originalParentNode.getNodeFrame(), initialTransformToParent);
         initialPose.changeFrame(getNodeFrame().getParent());
         initialPose.get(getNodeToParentFrameTransform());
      }
      else
      {
         getNodeToParentFrameTransform().set(initialTransformToParent);
      }
      getNodeFrame().update();
   }

   public String getVisualModelFilePath()
   {
      return visualModelFilePath;
   }

   public RigidBodyTransform getVisualModelToNodeFrameTransform()
   {
      return visualModelToNodeFrameTransform;
   }
}
