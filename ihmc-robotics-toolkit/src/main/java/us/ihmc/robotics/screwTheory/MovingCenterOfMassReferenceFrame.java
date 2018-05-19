package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * {@code MovingCenterOfMassReferenceFrame} is a reference frame that is centered at the center of
 * mass of a multi-rigid-body system attached to a {@code rootBody}.
 * <p>
 * In addition to providing the center of mass location as a reference frame, the
 * {@code MovingCenterOfMassReferenceFrame} can be used to obtain the twist of the center of mass
 * with its angular part being zero, i.e. no angular velocity.
 * </p>
 */
public class MovingCenterOfMassReferenceFrame extends MovingReferenceFrame
{
   private final CenterOfMassCalculator centerOfMassCalculator;
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final FramePoint3D centerOfMass;
   private final FrameVector3D centerOfMassVelocity;

   
   public MovingCenterOfMassReferenceFrame(String frameName, RigidBody rootBody)
   {
      this(frameName, rootBody.getBodyFixedFrame(), rootBody);
   }

   public MovingCenterOfMassReferenceFrame(String frameName, ReferenceFrame parentFrame, RigidBody rootBody)
   {
      super(frameName, rootBody.getBodyFixedFrame());

      centerOfMassCalculator = new CenterOfMassCalculator(rootBody, parentFrame);
      centerOfMassJacobian = new CenterOfMassJacobian(ScrewTools.computeSupportAndSubtreeSuccessors(rootBody), parentFrame);
      centerOfMass = new FramePoint3D(parentFrame);
      centerOfMassVelocity = new FrameVector3D(parentFrame);
   }

   @Override
   protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
   {
      centerOfMassJacobian.compute();
      centerOfMassJacobian.getCenterOfMassVelocity(centerOfMassVelocity);
      centerOfMassVelocity.changeFrame(this);
      twistRelativeToParentToPack.setToZero(this, parentFrame, this);
      twistRelativeToParentToPack.setLinearPart(centerOfMassVelocity);

   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      centerOfMassCalculator.compute();
      centerOfMassCalculator.getCenterOfMass(centerOfMass);
      transformToParent.setTranslationAndIdentityRotation(centerOfMass);
   }
}
