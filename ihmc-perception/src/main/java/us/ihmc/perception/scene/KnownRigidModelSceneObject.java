package us.ihmc.perception.scene;

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

   public KnownRigidModelSceneObject(String name, String visualModelFilePath)
   {
      super(name);
      this.visualModelFilePath = visualModelFilePath;
   }

   public String getVisualModelFilePath()
   {
      return visualModelFilePath;
   }
}
