package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class BodyFixedReferenceFrame extends ReferenceFrame
{
   private static final long serialVersionUID = -4710160817378608924L;
   private final RigidBody rigidBody;

   BodyFixedReferenceFrame(String nameSuffix, RigidBody rigidBody, ReferenceFrame parentFrame, RigidBodyTransform transformToParent, boolean isInertialFrame)
   {
      this(nameSuffix, rigidBody, parentFrame, transformToParent, isInertialFrame, false);
   }

   BodyFixedReferenceFrame(String nameSuffix, RigidBody rigidBody, ReferenceFrame parentFrame, RigidBodyTransform transformToParent, boolean isInertialFrame,
                           boolean isZUpFrame)
   {
      super(rigidBody.getName() + nameSuffix, parentFrame, transformToParent, isInertialFrame, isZUpFrame);
      this.rigidBody = rigidBody;
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
   }

   public RigidBody getRigidBody()
   {
      return rigidBody;
   }
}
