package us.ihmc.perception.sceneGraph.rigidBodies;

import gnu.trove.map.TLongObjectMap;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;

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
   private final TLongObjectMap<SceneNode> sceneGraphIDToNodeMap;
   private final long initialParentNodeID;
   private final RigidBodyTransform initialTransformToParent = new RigidBodyTransform();
   private transient final FramePose3D initialPose = new FramePose3D();
   private final String visualModelFilePath;
   private final RigidBodyTransform visualModelToNodeFrameTransform;

   public PredefinedRigidBodySceneNode(long id,
                                       String name,
                                       TLongObjectMap<SceneNode> sceneGraphIDToNodeMap,
                                       long initialParentNodeID,
                                       RigidBodyTransformReadOnly initialTransformToParent,
                                       String visualModelFilePath,
                                       RigidBodyTransform visualModelToNodeFrameTransform)
   {
      super(id, name);
      this.sceneGraphIDToNodeMap = sceneGraphIDToNodeMap;
      this.initialParentNodeID = initialParentNodeID;
      this.initialTransformToParent.set(initialTransformToParent);
      this.visualModelFilePath = visualModelFilePath;
      this.visualModelToNodeFrameTransform = visualModelToNodeFrameTransform;

      getNodeToParentFrameTransform().set(initialTransformToParent);
      getNodeFrame().update();
   }

   public void setTrackInitialParent(boolean trackInitialParent)
   {
      boolean previousTrackingInitialParent = getTrackingInitialParent();
      if (previousTrackingInitialParent != trackInitialParent)
      {
         SceneNode rootNode = sceneGraphIDToNodeMap.get(SceneGraph.ROOT_NODE_ID);
         SceneNode initialParentNode = sceneGraphIDToNodeMap.get(initialParentNodeID);
         if (trackInitialParent)
         {
            rootNode.getChildren().remove(this);
            initialParentNode.getChildren().add(this);
            ensureFramesMatchParentsRecursively(initialParentNode.getNodeFrame());
         }
         else
         {
            initialParentNode.getChildren().remove(this);
            rootNode.getChildren().add(this);
            ensureFramesMatchParentsRecursively(rootNode.getNodeFrame());
         }
      }
   }

   public boolean getTrackingInitialParent()
   {
      return getNodeFrame().getParent() == sceneGraphIDToNodeMap.get(initialParentNodeID).getNodeFrame();
   }

   /**
    * This sets the transform to the parent node back to the original one.
    * This is robust to whether or not this node is currently tracking the detected pose.
    */
   public void clearOffset()
   {
      SceneNode initialParentNode = sceneGraphIDToNodeMap.get(initialParentNodeID);
      if (getNodeFrame().getParent() != initialParentNode.getNodeFrame())
      {
         SceneNode originalParentNode = initialParentNode;
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

   public TLongObjectMap<SceneNode> getSceneGraphIDToNodeMap()
   {
      return sceneGraphIDToNodeMap;
   }

   public long getInitialParentNodeID()
   {
      return initialParentNodeID;
   }

   public RigidBodyTransform getInitialTransformToParent()
   {
      return initialTransformToParent;
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
