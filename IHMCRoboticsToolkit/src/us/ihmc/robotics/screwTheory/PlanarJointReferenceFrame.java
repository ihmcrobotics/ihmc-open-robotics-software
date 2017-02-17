package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FrameOrientation2d;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class PlanarJointReferenceFrame extends ReferenceFrame
{
   private static final long serialVersionUID = 1179880149506811812L;
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

   public void setOrientationAndUpdate(FrameOrientation2d orientation)
   {
      orientation.checkReferenceFrameMatch(parentFrame);
      this.rotation = orientation.getYaw();
      this.update();
   }

   public void setTranslationAndUpdate(FrameVector2d translation)
   {
      translation.checkReferenceFrameMatch(parentFrame);
      this.translation.setX(translation.getX());
      this.translation.setY(translation.getY());
      this.update();
   }

   public void setFramePose2DAndUpdate(FramePose2d framePose2d)
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
