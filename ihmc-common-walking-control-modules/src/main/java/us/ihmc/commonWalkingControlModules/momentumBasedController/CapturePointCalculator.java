package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
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

      CapturePointTools.computeCapturePointPosition(centerOfMassPosition2d, centerOfMassVelocity2d, omega0, capturePointToPack);
   }

   public static double computeOmega0ConstantHeight(double g, double z)
   {
      return Math.sqrt(g / z);
   }
}
