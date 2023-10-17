package us.ihmc.perception.sceneGraph.rigidBody;

import gnu.trove.map.TLongObjectMap;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.perception.sceneGraph.SceneNode;

/**
 * A scene object that is a rigid body whose shape and appearance is
 * known beforehand.
 * <p>
 * Rigid bodies as in a door panel, chair, can of soup, etc.
 * <p>
 * This class also provides support for remembering the parent frame
 * and initial transform to parent, allowing an operator to manually
 * adjust it and also reset it.
 * <p>
 * TODO:
 *   - Add collision information
 */
public class PredefinedRigidBodySceneNode extends RigidBodySceneNode
{
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
      super(id, name, sceneGraphIDToNodeMap, initialParentNodeID, initialTransformToParent);
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
}
