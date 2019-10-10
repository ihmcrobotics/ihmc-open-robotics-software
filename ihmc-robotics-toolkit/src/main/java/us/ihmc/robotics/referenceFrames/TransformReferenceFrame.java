package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public class TransformReferenceFrame extends ReferenceFrame
{
   public final RigidBodyTransform transform3D = new RigidBodyTransform();

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
      transformToParent.set(this.transform3D);
   }

   public void setTransformAndUpdate(RigidBodyTransformReadOnly transform)
   {
      this.transform3D.set(transform);
      this.update();
   }
}


