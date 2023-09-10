package us.ihmc.perception.sceneGraph.rigidBodies;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;

import java.util.function.Supplier;

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
   private Supplier<SceneNode> originalParentNodeSupplier;
   private Supplier<SceneNode> rootNodeSupplier;
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

   public void setOriginalParentID(SceneGraph sceneGraph, long originalParentID)
   {
      this.originalParentID = originalParentID;
      originalParentNodeSupplier = () -> sceneGraph.getIDToNodeMap().get(originalParentID);
      rootNodeSupplier = sceneGraph::getRootNode;
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
      if (getNodeFrame().getParent() != rootNodeSupplier.get().getNodeFrame())
      {
         SceneNode originalParentNode = originalParentNodeSupplier.get();
         originalPose.setIncludingFrame(originalParentNode.getNodeFrame(), originalTransformToParent);
         originalPose.changeFrame(getNodeFrame().getParent());
         originalPose.get(getNodeToParentFrameTransform());
      }
      else
      {
         getNodeToParentFrameTransform().set(originalTransformToParent);
      }
      getNodeFrame().update();
   }
}
