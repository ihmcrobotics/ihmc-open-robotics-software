package us.ihmc.perception.sceneGraph;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;

/**
 * Represents a node on the Scene Knowledge Graph (to be defined).
 *
 * We give each node a name and a reference frame.
 */
public abstract class SceneNode
{
   private final String name;
   private final ModifiableReferenceFrame referenceFrame;

   public SceneNode(String name)
   {
      this(name, ReferenceFrame.getWorldFrame());
   }

   public SceneNode(String name, ReferenceFrame parentFrame)
   {
      this.name = name;
      this.referenceFrame = new ModifiableReferenceFrame(parentFrame);
   }

   public String getName()
   {
      return name;
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame.getReferenceFrame();
   }

   /**
    * Used to get and set the transform to the parent frame.
    * If you modify this transform, you must then call {@link ReferenceFrame#update()} on {@link #getReferenceFrame()}.
    * @return the transform to the parent frame
    */
   public RigidBodyTransform getTransformToParent()
   {
      return referenceFrame.getTransformToParent();
   }
}
