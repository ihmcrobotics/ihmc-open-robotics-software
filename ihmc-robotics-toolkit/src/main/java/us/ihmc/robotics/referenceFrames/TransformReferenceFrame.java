package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public class TransformReferenceFrame extends ReferenceFrame
{
   private final RigidBodyTransform transformToParent = new RigidBodyTransform();

   public TransformReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame);
   }

   public TransformReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBodyTransform transformToParent)
   {
      this(frameName, parentFrame);
      setTransformAndUpdate(transformToParent);
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      transformToParent.set(this.transformToParent);
   }

   public void setTransformAndUpdate(RigidBodyTransformReadOnly transform)
   {
      this.transformToParent.set(transform);
      this.update();
   }
}


