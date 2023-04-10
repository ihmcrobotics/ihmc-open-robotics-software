package us.ihmc.perception.scene;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;

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
}
