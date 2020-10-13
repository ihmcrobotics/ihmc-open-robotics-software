package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
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
   
   public void getCenterOfMassPosition(Point3DBasics centerOfMassPositionToSet)
   {
      centerOfMassPositionToSet.set(centerOfMassPosition);
   }
   
   public void getCenterOfMassVelocity(Vector3DBasics centerOfMassVelocityToSet)
   {
      centerOfMassVelocityToSet.set(centerOfMassVelocity);
   }

   public CenterOfMassJacobian getCenterOfMassJacobian()
   {
      return centerOfMassJacobian;
   }

   /**
    * Deprecated on 6/1/2020.
    *
    * Use {@link CapturePointTools#computeCapturePointPosition(FramePoint2DReadOnly, FrameVector2DReadOnly, double, FixedFramePoint2DBasics)} instead
    */
   @Deprecated
   public static void computeCapturePoint(FramePoint2DBasics capturePointToPack, FramePoint2DReadOnly centerOfMassInWorld,
                                          FrameVector2DReadOnly centerOfMassVelocityInWorld, double omega0)
   {
      CapturePointTools.computeCapturePointPosition(centerOfMassInWorld, centerOfMassVelocityInWorld, omega0, capturePointToPack);
   }

   /**
    * Deprecated on 6/1/2020.
    *
    * Use {@link CapturePointTools#computeCapturePointVelocity(FrameVector2DReadOnly, FrameVector2DReadOnly, double, FixedFrameVector2DBasics)} instead
    */
   @Deprecated
   public static void computeCapturePointVelocity(FrameVector2DBasics capturePointVelocityToPack, FrameVector2DReadOnly centerOfMassVelocityInWorld,
                                                  FrameVector2DReadOnly centerOfMassAccelerationInWorld, double omega0)
   {
      CapturePointTools.computeCapturePointVelocity(centerOfMassVelocityInWorld, centerOfMassAccelerationInWorld, omega0, capturePointVelocityToPack);
   }

   /**
    * Deprecated on 6/1/2020.
    *
    * Use {@link CapturePointTools#computeCapturePointVelocity(FrameVector3DReadOnly, FrameVector3DReadOnly, double, FixedFrameVector3DBasics)} instead
    */
   @Deprecated
   public static void computeDCMVelocity(FrameVector3DBasics capturePointVelocityToPack, FrameVector3DReadOnly centerOfMassVelocityInWorld,
                                         FrameVector3DReadOnly centerOfMassAccelerationInWorld, double omega0)
   {
      CapturePointTools.computeCapturePointVelocity(centerOfMassVelocityInWorld, centerOfMassAccelerationInWorld, omega0, capturePointVelocityToPack);
   }

   public static double computeOmega0ConstantHeight(double g, double z)
   {
      return Math.sqrt(g / z);
   }
}
