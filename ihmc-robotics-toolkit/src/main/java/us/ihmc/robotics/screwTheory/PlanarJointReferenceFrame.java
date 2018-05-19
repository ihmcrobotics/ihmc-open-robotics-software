package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.referenceFrame.FrameOrientation2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class PlanarJointReferenceFrame extends ReferenceFrame
{
   private double rotation = 0.0;
   private final Vector3D translation = new Vector3D();

   public PlanarJointReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame);
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      transformToParent.setRotationYawAndZeroTranslation(rotation);
      transformToParent.setTranslation(translation);
   }

   public void setRotationAndUpdate(double rotation)
   {
      this.rotation = rotation;
      this.update();
   }

   public void setOrientationAndUpdate(FrameOrientation2D orientation)
   {
      orientation.checkReferenceFrameMatch(parentFrame);
      this.rotation = orientation.getYaw();
      this.update();
   }

   public void setTranslationAndUpdate(FrameVector2D translation)
   {
      translation.checkReferenceFrameMatch(parentFrame);
      this.translation.setX(translation.getX());
      this.translation.setY(translation.getY());
      this.update();
   }

   public void setFramePose2DAndUpdate(FramePose2D framePose2d)
   {
      framePose2d.checkReferenceFrameMatch(parentFrame);
      this.rotation = framePose2d.getYaw();
      this.translation.setX(framePose2d.getX());
      this.translation.setY(framePose2d.getY());
      this.update();
   }
   
   public double getRotation()
   {
      return rotation;
   }
}
