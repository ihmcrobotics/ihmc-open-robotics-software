package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class PrismaticJointReferenceFrame extends OneDoFJointReferenceFrame
{
   private static final long serialVersionUID = -1982346476164458546L;
   private final Vector3D axis;
   private double position; // TODO: YoVariablize
   private Vector3D tempTranslation = new Vector3D();

   public PrismaticJointReferenceFrame(String frameName, ReferenceFrame parentFrame, FrameVector axis)
   {
      super(frameName, parentFrame);
      axis.checkReferenceFrameMatch(parentFrame);
      this.axis = axis.getVectorCopy();
   }

   @Override
   public void setAndUpdate(double jointPosition)
   {
      this.position = jointPosition;
      this.update();
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      tempTranslation.set(axis);
      tempTranslation.scale(position);
      transformToParent.setTranslationAndIdentityRotation(tempTranslation);
   }
}
