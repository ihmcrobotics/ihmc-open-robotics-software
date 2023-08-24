package us.ihmc.perception.sceneGraph;

import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * A scene object that is a rigid body whose shape and appearance is
 * known beforehand.
 *
 * Rigid bodies as in a door panel, chair, can of soup, etc.
 *
 * TODO:
 *   - Add collision information
 */
public abstract class PredefinedRigidBodySceneNode extends DetectableSceneNode
{
   private final String visualModelFilePath;
   private final RigidBodyTransform visualModelToNodeFrameTransform;

   public PredefinedRigidBodySceneNode(String name, String visualModelFilePath, RigidBodyTransform visualModelToNodeFrameTransform)
   {
      super(name);
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
