package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.transform.RigidBodyTransform;

public class TransformReferenceFrame extends ReferenceFrame
{
   private static final long serialVersionUID = -6741627181585210414L;
   public final RigidBodyTransform transform3D = new RigidBodyTransform();

   public TransformReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame, false, false, false);
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

   public void setTransformAndUpdate(RigidBodyTransform transform)
   {
      this.transform3D.set(transform);
      this.update();
   }
}


