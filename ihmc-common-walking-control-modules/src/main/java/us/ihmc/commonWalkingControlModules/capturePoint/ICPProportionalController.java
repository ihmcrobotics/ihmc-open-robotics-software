package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ICPProportionalController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FrameVector2D tempControl = new FrameVector2D(worldFrame);
   private final YoFrameVector2d icpError = new YoFrameVector2d("icpError", "", worldFrame, registry);
   private final YoFrameVector2d icpErrorIntegrated = new YoFrameVector2d("icpErrorIntegrated", "", worldFrame, registry);

   private final YoFrameVector2d feedbackPart = new YoFrameVector2d("feedbackPart", "", worldFrame, registry);

   private final YoFramePoint2d cmpOutput = new YoFramePoint2d("icpControlCMPOutput", "", worldFrame, registry);
   private final YoFramePoint2d rateLimitedCMPOutput;
   private final boolean rateLimitFeedbackPart;
   private final DoubleProvider feedbackPartMaxRate;

   private final YoFramePoint icpPosition;
   private final FrameVector2D icpIntegral = new FrameVector2D(worldFrame);

   private final double controlDT;
   private final DoubleProvider captureKpParallelToMotion;
   private final DoubleProvider captureKpOrthogonalToMotion;

   private final DoubleProvider captureKi;
   private final DoubleProvider captureKiBleedoff;

   private final Vector2dZUpFrame icpVelocityDirectionFrame;

   private final FrameVector2D tempICPErrorIntegrated = new FrameVector2D(worldFrame);

   public ICPProportionalController(ICPControlGainsProvider gains, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.controlDT = controlDT;

      icpVelocityDirectionFrame = new Vector2dZUpFrame("icpVelocityDirectionFrame", worldFrame);

      icpPosition = new YoFramePoint("icpPosition", ReferenceFrame.getWorldFrame(), registry);
      parentRegistry.addChild(registry);

      captureKpParallelToMotion = gains.getYoKpParallelToMotion();
      captureKpOrthogonalToMotion = gains.getYoKpOrthogonalToMotion();
      captureKi = gains.getYoKi();
      captureKiBleedoff = gains.getYoKiBleedOff();

      feedbackPartMaxRate = gains.getFeedbackPartMaxRate();
      rateLimitFeedbackPart = feedbackPartMaxRate != null;
      if (rateLimitFeedbackPart)
         rateLimitedCMPOutput = new YoFramePoint2d("icpControlRateLimitedCMPOutput", "", worldFrame, registry);
      else
         rateLimitedCMPOutput = null;
   }

   public void reset()
   {
      icpErrorIntegrated.set(0.0, 0.0);
   }

   private final FramePoint2D desiredCMP = new FramePoint2D();
   private final FramePoint2D previousPerfectCMP = new FramePoint2D();

   public FramePoint2D doProportionalControl(FramePoint2D desiredCMPPreviousValue, FramePoint2D capturePoint, FramePoint2D desiredCapturePoint,
         FramePoint2D finalDesiredCapturePoint, FrameVector2D desiredCapturePointVelocity, FramePoint2D perfectCMP, double omega0)
   {
      capturePoint.changeFrame(worldFrame);
      desiredCapturePoint.changeFrame(worldFrame);
      finalDesiredCapturePoint.changeFrame(worldFrame);
      desiredCapturePointVelocity.changeFrame(worldFrame);

      desiredCMP.setIncludingFrame(capturePoint);

      icpPosition.set(capturePoint.getX(), capturePoint.getY(), 0.0);
      // feed forward part
      tempControl.setIncludingFrame(desiredCapturePointVelocity);
      tempControl.scale(1.0 / omega0);

      desiredCMP.sub(tempControl);

      // feedback part
      icpError.set(capturePoint);
      icpError.sub(desiredCapturePoint);

      tempControl.set(icpError);
      double epsilonZeroICPVelocity = 1e-5;
      if (desiredCapturePointVelocity.lengthSquared() > MathTools.square(epsilonZeroICPVelocity))
      {
         icpVelocityDirectionFrame.setXAxis(desiredCapturePointVelocity);
         tempControl.changeFrame(icpVelocityDirectionFrame);
         tempControl.setX(tempControl.getX() * captureKpParallelToMotion.getValue());
         tempControl.setY(tempControl.getY() * captureKpOrthogonalToMotion.getValue());
         tempControl.changeFrame(desiredCMP.getReferenceFrame());
      }
      else
      {
         tempControl.scale(captureKpOrthogonalToMotion.getValue());
      }

      tempICPErrorIntegrated.set(icpError);
      tempICPErrorIntegrated.scale(controlDT);
      tempICPErrorIntegrated.scale(captureKi.getValue());

      icpErrorIntegrated.scale(captureKiBleedoff.getValue());
      icpErrorIntegrated.add(tempICPErrorIntegrated);

      double length = icpErrorIntegrated.length();
      double maxLength = 0.02;
      if (length > maxLength)
      {
         icpErrorIntegrated.scale(maxLength / length);
      }

      if (Math.abs(captureKi.getValue()) < 1e-10)
      {
         icpErrorIntegrated.set(0.0, 0.0);
      }

      icpIntegral.set(icpErrorIntegrated);
      tempControl.add(icpIntegral);

      feedbackPart.set(tempControl);
      desiredCMP.add(tempControl);

      desiredCMP.changeFrame(cmpOutput.getReferenceFrame());
      cmpOutput.set(desiredCMP);

      if (rateLimitFeedbackPart)
      {
         rateLimitCMP(desiredCMP, desiredCMPPreviousValue, perfectCMP, previousPerfectCMP);
         rateLimitedCMPOutput.set(desiredCMP);
         previousPerfectCMP.set(perfectCMP);
      }

      return desiredCMP;
   }

   private final FrameVector2D cmpError = new FrameVector2D();
   private final FrameVector2D cmpErrorPreviousValue = new FrameVector2D();
   private final FrameVector2D cmpErrorDifference = new FrameVector2D();

   private void rateLimitCMP(FramePoint2D cmp, FramePoint2D cmpPreviousValue, FramePoint2D perfectCMP, FramePoint2D previousPerfectCMP)
   {
      if (feedbackPartMaxRate.getValue() < 1.0e-3)
         return;

      cmpError.setToZero(cmp.getReferenceFrame());
      cmpError.sub(cmp, perfectCMP);

      cmpErrorPreviousValue.setToZero(cmp.getReferenceFrame());
      cmpErrorPreviousValue.sub(cmpPreviousValue, previousPerfectCMP);

      cmpErrorDifference.sub(cmpError, cmpErrorPreviousValue);
      double errorDifferenceMagnitude = cmpErrorDifference.length();
      double errorDifferenceMax = controlDT * feedbackPartMaxRate.getValue();
      if (errorDifferenceMagnitude > errorDifferenceMax)
         cmpErrorDifference.scale(errorDifferenceMax / errorDifferenceMagnitude);

      cmpError.add(cmpErrorPreviousValue, cmpErrorDifference);
      cmp.add(perfectCMP, cmpError);
   }

   private class Vector2dZUpFrame extends ReferenceFrame
   {
      private final FrameVector2D xAxis;
      private final Vector3D x = new Vector3D();
      private final Vector3D y = new Vector3D();
      private final Vector3D z = new Vector3D();
      private final RotationMatrix rotation = new RotationMatrix();

      public Vector2dZUpFrame(String string, ReferenceFrame parentFrame)
      {
         super(string, parentFrame);
         xAxis = new FrameVector2D(parentFrame);
      }

      public void setXAxis(FrameVector2D xAxis)
      {
         this.xAxis.setIncludingFrame(xAxis);
         this.xAxis.changeFrame(parentFrame);
         this.xAxis.normalize();
         update();
      }

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         x.set(xAxis.getX(), xAxis.getY(), 0.0);
         z.set(0.0, 0.0, 1.0);
         y.cross(z, x);

         rotation.setColumns(x, y, z);

         transformToParent.setRotationAndZeroTranslation(rotation);
      }
   }

   public void bleedOffIntegralTerm()
   {
      icpErrorIntegrated.scale(0.9); //Bleed off quickly when projecting. 0.9 is a pretty arbitrary magic number.
   }
}
