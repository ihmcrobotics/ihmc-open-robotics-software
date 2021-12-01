package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class VolatileReferenceFrame extends ReferenceFrame
{
   private final RigidBodyTransform transformToParent;

   public VolatileReferenceFrame(String name, ReferenceFrame parentFrame, RigidBodyTransform transformToParent)
   {
      super(name, parentFrame);
      this.transformToParent = transformToParent;
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParentToUpdate)
   {
      transformToParentToUpdate.set(transformToParent);
   }

   /**
    * Returns a modifiable reference to this reference frame's transform to parent.
    * Changes to this object take effect on update().
    * <p>
    * This transform can be applied to a vector defined in this frame in order to obtain the equivalent
    * vector in the parent frame.
    * </p>
    *
    * @return the transform to the parent frame.
    */
   @Override
   public RigidBodyTransform getTransformToParent()
   {
      return transformToParent;
   }
}
