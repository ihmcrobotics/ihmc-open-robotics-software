package us.ihmc.perception.sceneGraph.rigidBody.primitive;

import gnu.trove.map.TLongObjectMap;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.RigidBodySceneNode;

/**
 * A scene object that is a rigid body whose shape can be reshaped and resized.
 * <p>
 * Rigid bodies are represented by primitive shapes.
 * <p>
 * This class also provides support for remembering the parent frame
 * and initial transform to parent, allowing an operator to manually
 * adjust it and also reset it.
 * <p>
 * TODO:
 *   - Add collision information
 */
public class PrimitiveRigidBodySceneNode extends RigidBodySceneNode
{
   private final PrimitiveRigidBodyShape shape;

   public PrimitiveRigidBodySceneNode(long id,
                                      String name,
                                      TLongObjectMap<SceneNode> sceneGraphIDToNodeMap,
                                      long initialParentNodeID,
                                      RigidBodyTransformReadOnly initialTransformToParent,
                                      PrimitiveRigidBodyShape shape)
   {
      super(id, name, sceneGraphIDToNodeMap, initialParentNodeID, initialTransformToParent);
      this.shape = shape;
   }

   public PrimitiveRigidBodyShape getShape()
   {
      return shape;
   }
}
