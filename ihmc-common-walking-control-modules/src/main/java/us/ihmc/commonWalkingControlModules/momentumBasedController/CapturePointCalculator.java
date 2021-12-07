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
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.humanoidRobotics.model.CenterOfMassStateProvider;
import us.ihmc.mecano.algorithms.CenterOfMassJacobian;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;

public class CapturePointCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final CenterOfMassStateProvider centerOfMassStateProvider;

   private final FramePoint3D centerOfMassPosition = new FramePoint3D();
   private final FrameVector3D centerOfMassVelocity = new FrameVector3D();
   private final FramePoint2D centerOfMassPosition2d = new FramePoint2D();
   private final FrameVector2D centerOfMassVelocity2d = new FrameVector2D();

   public CapturePointCalculator(ReferenceFrame centerOfMassFrame, RigidBodyReadOnly elevator)
   {
      this(new CenterOfMassStateProvider()
      {
         FramePoint3D position = new FramePoint3D();
         CenterOfMassJacobian centerOfMassJacobian = new CenterOfMassJacobian(elevator, worldFrame);

         @Override
         public FramePoint3DReadOnly getCenterOfMassPosition()
         {
            position.setToZero(centerOfMassFrame);
            return position;
         }

         @Override
         public FrameVector3DReadOnly getCenterOfMassVelocity()
         {
            centerOfMassJacobian.reset();
            return centerOfMassJacobian.getCenterOfMassVelocity();
         }
      });
   }

   public CapturePointCalculator(CenterOfMassStateProvider centerOfMassStateProvider)
   {
      this.centerOfMassStateProvider = centerOfMassStateProvider;
   }

   public void compute(FramePoint2DBasics capturePointToPack, double omega0)
   {
      centerOfMassPosition.setIncludingFrame(centerOfMassStateProvider.getCenterOfMassPosition());
      centerOfMassVelocity.setIncludingFrame(centerOfMassStateProvider.getCenterOfMassVelocity());

      centerOfMassPosition.changeFrame(worldFrame);
      centerOfMassVelocity.changeFrame(worldFrame);

      centerOfMassPosition2d.setIncludingFrame(centerOfMassPosition);
      centerOfMassVelocity2d.setIncludingFrame(centerOfMassVelocity);

      CapturePointTools.computeCapturePointPosition(centerOfMassPosition2d, centerOfMassVelocity2d, omega0, capturePointToPack);
   }

   public FramePoint3DReadOnly getCenterOfMassPosition()
   {
      return centerOfMassPosition;
   }

   public FrameVector3DReadOnly getCenterOfMassVelocity()
   {
      return centerOfMassVelocity;
   }

   /**
    * Deprecated on 6/1/2020. Use
    * {@link CapturePointTools#computeCapturePointPosition(FramePoint2DReadOnly, FrameVector2DReadOnly, double, FixedFramePoint2DBasics)}
    * instead
    */
   @Deprecated
   public static void computeCapturePoint(FramePoint2DBasics capturePointToPack,
                                          FramePoint2DReadOnly centerOfMassInWorld,
                                          FrameVector2DReadOnly centerOfMassVelocityInWorld,
                                          double omega0)
   {
      CapturePointTools.computeCapturePointPosition(centerOfMassInWorld, centerOfMassVelocityInWorld, omega0, capturePointToPack);
   }

   /**
    * Deprecated on 6/1/2020. Use
    * {@link CapturePointTools#computeCapturePointVelocity(FrameVector2DReadOnly, FrameVector2DReadOnly, double, FixedFrameVector2DBasics)}
    * instead
    */
   @Deprecated
   public static void computeCapturePointVelocity(FrameVector2DBasics capturePointVelocityToPack,
                                                  FrameVector2DReadOnly centerOfMassVelocityInWorld,
                                                  FrameVector2DReadOnly centerOfMassAccelerationInWorld,
                                                  double omega0)
   {
      CapturePointTools.computeCapturePointVelocity(centerOfMassVelocityInWorld, centerOfMassAccelerationInWorld, omega0, capturePointVelocityToPack);
   }

   /**
    * Deprecated on 6/1/2020. Use
    * {@link CapturePointTools#computeCapturePointVelocity(FrameVector3DReadOnly, FrameVector3DReadOnly, double, FixedFrameVector3DBasics)}
    * instead
    */
   @Deprecated
   public static void computeDCMVelocity(FrameVector3DBasics capturePointVelocityToPack,
                                         FrameVector3DReadOnly centerOfMassVelocityInWorld,
                                         FrameVector3DReadOnly centerOfMassAccelerationInWorld,
                                         double omega0)
   {
      CapturePointTools.computeCapturePointVelocity(centerOfMassVelocityInWorld, centerOfMassAccelerationInWorld, omega0, capturePointVelocityToPack);
   }

   public static double computeOmega0ConstantHeight(double g, double z)
   {
      return Math.sqrt(g / z);
   }
}
