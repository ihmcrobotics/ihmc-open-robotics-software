package us.ihmc.perception.sceneGraph;

import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * A scene object that is a rigid body whose shape and appearance is
 * known beforehand.
 *
 * TODO:
 *   - Add collision information?
 */
public class KnownRigidModelSceneObject extends DetectableSceneObject
{
   private final String visualModelFilePath;
   private final RigidBodyTransform visualModelToNodeFrameTransform;

   public KnownRigidModelSceneObject(String name, String visualModelFilePath, RigidBodyTransform visualModelToNodeFrameTransform)
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
