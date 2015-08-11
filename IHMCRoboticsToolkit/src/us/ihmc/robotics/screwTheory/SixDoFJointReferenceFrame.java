package us.ihmc.robotics.screwTheory;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

public class SixDoFJointReferenceFrame extends ReferenceFrame
{
   private static final long serialVersionUID = 8238436678986979988L;
   private final Quat4d rotation = new Quat4d();
   private final Vector3d translation = new Vector3d();

   public SixDoFJointReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame);
   }

   public void setRotation(Quat4d rotation)
   {
      this.rotation.set(rotation);
   }

   public void setTranslation(Vector3d translation)
   {
      this.translation.set(translation);
   }

   public void getRotation(Quat4d rotationToPack)
   {
      rotationToPack.set(rotation);
   }

   public void getTraslation(Tuple3d translationToPack)
   {
      translationToPack.set(translation);
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      transformToParent.set(rotation, translation);
   }
}
