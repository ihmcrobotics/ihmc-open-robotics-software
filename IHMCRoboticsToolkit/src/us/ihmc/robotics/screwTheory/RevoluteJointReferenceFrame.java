package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class RevoluteJointReferenceFrame extends OneDoFJointReferenceFrame
{
   private static final long serialVersionUID = -1982346476164458546L;
   private final Vector3D axis;
   private double angle;
   private final AxisAngle tempAxisAngle = new AxisAngle();

   public RevoluteJointReferenceFrame(String frameName, ReferenceFrame parentFrame, FrameVector axis)
   {
      super(frameName, parentFrame);
      axis.checkReferenceFrameMatch(parentFrame);
      this.axis = axis.getVectorCopy();
   }

   @Override
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