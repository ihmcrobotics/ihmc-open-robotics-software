package us.ihmc.robotics.screwTheory;

import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Vector3d;

public class RevoluteJointReferenceFrame extends OneDoFJointReferenceFrame
{
   private static final long serialVersionUID = -1982346476164458546L;
   private final Vector3d axis;
   private double angle;
   private final AxisAngle4d tempAxisAngle = new AxisAngle4d();

   public RevoluteJointReferenceFrame(String frameName, ReferenceFrame parentFrame, FrameVector axis)
   {
      super(frameName, parentFrame);
      axis.checkReferenceFrameMatch(parentFrame);
      this.axis = axis.getVectorCopy();
   }

   public void setAndUpdate(double jointPosition)
   {
      this.angle = jointPosition;
      this.update();
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      tempAxisAngle.set(axis, angle);
      transformToParent.setRotationAndZeroTranslation(tempAxisAngle);
   }
}
