package us.ihmc.perception.scene;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;

/**
 * Represents an object node on the Scene Knowledge Graph (to be defined).
 *
 * We give each object a name, unique identifier, and a reference frame.
 *
 * Object as in a door, chair, can of soup, etc.
 */
public abstract class SceneObject
{
   private final String name;
   private final long uuid;
   private final ModifiableReferenceFrame referenceFrame;

   public SceneObject(String name, long uuid)
   {
      this(name, uuid, ReferenceFrame.getWorldFrame());
   }

   public SceneObject(String name, long uuid, ReferenceFrame parentFrame)
   {
      this.name = name;
      this.uuid = uuid;
      this.referenceFrame = new ModifiableReferenceFrame(parentFrame);
   }

   public String getName()
   {
      return name;
   }

   public long getUUID()
   {
      return uuid;
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame.getReferenceFrame();
   }

   public RigidBodyTransform getTransformToParent()
   {
      return referenceFrame.getTransformToParent();
   }
}
