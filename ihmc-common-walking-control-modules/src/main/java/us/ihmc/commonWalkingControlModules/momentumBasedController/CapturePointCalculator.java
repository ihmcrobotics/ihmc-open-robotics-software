package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public class CapturePointCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ReferenceFrame centerOfMassFrame;
   private final CenterOfMassJacobian centerOfMassJacobian;

   private final FramePoint3D centerOfMassPosition = new FramePoint3D();
   private final FrameVector3D centerOfMassVelocity = new FrameVector3D();
   private final FramePoint2D centerOfMassPosition2d = new FramePoint2D();
   private final FrameVector2D centerOfMassVelocity2d = new FrameVector2D();

   public CapturePointCalculator(ReferenceFrame centerOfMassFrame, RigidBodyBasics elevator)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      this.centerOfMassJacobian = new CenterOfMassJacobian(elevator, worldFrame);
   }

   public void compute(FramePoint2DBasics capturePointToPack, double omega0)
   {
      centerOfMassJacobian.reset();
      centerOfMassPosition.setToZero(centerOfMassFrame);
      centerOfMassVelocity.setIncludingFrame(centerOfMassJacobian.getCenterOfMassVelocity());

      centerOfMassPosition.changeFrame(worldFrame);
      centerOfMassVelocity.changeFrame(worldFrame);

      centerOfMassPosition2d.setIncludingFrame(centerOfMassPosition);
      centerOfMassVelocity2d.setIncludingFrame(centerOfMassVelocity);

      CapturePointCalculator.computeCapturePoint(capturePointToPack, centerOfMassPosition2d, centerOfMassVelocity2d, omega0);
   }

   public static void computeCapturePoint(FramePoint2DBasics capturePointToPack, FramePoint2DReadOnly centerOfMassInWorld,
                                          FrameVector2DReadOnly centerOfMassVelocityInWorld, double omega0)
   {
      centerOfMassInWorld.checkReferenceFrameMatch(worldFrame);
      centerOfMassVelocityInWorld.checkReferenceFrameMatch(worldFrame);

      capturePointToPack.setToZero(worldFrame);
      capturePointToPack.set(centerOfMassVelocityInWorld);
      capturePointToPack.scale(1.0 / omega0);
      capturePointToPack.add(centerOfMassInWorld);
   }

   public static void computeCapturePointVelocity(FrameVector2DBasics capturePointVelocityToPack, FrameVector2DReadOnly centerOfMassVelocityInWorld,
                                                  FrameVector2DReadOnly centerOfMassAccelerationInWorld, double omega0)
   {
      centerOfMassVelocityInWorld.checkReferenceFrameMatch(worldFrame);
      centerOfMassAccelerationInWorld.checkReferenceFrameMatch(worldFrame);

      capturePointVelocityToPack.setToZero(worldFrame);
      capturePointVelocityToPack.set(centerOfMassAccelerationInWorld);
      capturePointVelocityToPack.scale(1.0 / omega0);
      capturePointVelocityToPack.add(centerOfMassVelocityInWorld);
   }

   public static double computeOmega0ConstantHeight(double g, double z)
   {
      return Math.sqrt(g / z);
   }
}
