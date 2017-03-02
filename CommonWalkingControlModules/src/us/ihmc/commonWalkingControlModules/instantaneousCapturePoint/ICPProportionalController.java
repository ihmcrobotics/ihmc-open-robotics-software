package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ICPProportionalController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FrameVector2d tempControl = new FrameVector2d(worldFrame);
   private final YoFrameVector2d icpError = new YoFrameVector2d("icpError", "", worldFrame, registry);
   private final YoFrameVector2d icpErrorIntegrated = new YoFrameVector2d("icpErrorIntegrated", "", worldFrame, registry);

   private final YoFrameVector2d feedbackPart = new YoFrameVector2d("feedbackPart", "", worldFrame, registry);

   private final YoFramePoint2d cmpOutput = new YoFramePoint2d("icpControlCMPOutput", "", worldFrame, registry);
   private final YoFramePoint2d rateLimitedCMPOutput;
   private final boolean rateLimitFeedbackPart;
   private final DoubleYoVariable feedbackPartMaxRate;

   private final YoFramePoint icpPosition;
   private final FrameVector2d icpIntegral = new FrameVector2d(worldFrame);

   private final double controlDT;
   private final DoubleYoVariable captureKpParallelToMotion;
   private final DoubleYoVariable captureKpOrthogonalToMotion;

   private final DoubleYoVariable captureKi;
   private final DoubleYoVariable captureKiBleedoff;

   private final Vector2dZUpFrame icpVelocityDirectionFrame;

   private final FrameVector2d tempICPErrorIntegrated = new FrameVector2d(worldFrame);

   public ICPProportionalController(ICPControlGains gains, double controlDT, YoVariableRegistry parentRegistry)
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

   private final FramePoint2d desiredCMP = new FramePoint2d();
   private final FramePoint2d previousPerfectCMP = new FramePoint2d();

   public FramePoint2d doProportionalControl(FramePoint2d desiredCMPPreviousValue, FramePoint2d capturePoint, FramePoint2d desiredCapturePoint,
         FramePoint2d finalDesiredCapturePoint, FrameVector2d desiredCapturePointVelocity, FramePoint2d perfectCMP, double omega0)
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

      icpError.getFrameTuple2d(tempControl);
      double epsilonZeroICPVelocity = 1e-5;
      if (desiredCapturePointVelocity.lengthSquared() > MathTools.square(epsilonZeroICPVelocity))
      {
         icpVelocityDirectionFrame.setXAxis(desiredCapturePointVelocity);
         tempControl.changeFrame(icpVelocityDirectionFrame);
         tempControl.setX(tempControl.getX() * captureKpParallelToMotion.getDoubleValue());
         tempControl.setY(tempControl.getY() * captureKpOrthogonalToMotion.getDoubleValue());
         tempControl.changeFrame(desiredCMP.getReferenceFrame());
      }
      else
      {
         tempControl.scale(captureKpOrthogonalToMotion.getDoubleValue());
      }

      icpError.getFrameTuple2d(tempICPErrorIntegrated);
      tempICPErrorIntegrated.scale(controlDT);
      tempICPErrorIntegrated.scale(captureKi.getDoubleValue());

      icpErrorIntegrated.scale(captureKiBleedoff.getDoubleValue());
      icpErrorIntegrated.add(tempICPErrorIntegrated);

      double length = icpErrorIntegrated.length();
      double maxLength = 0.02;
      if (length > maxLength)
      {
         icpErrorIntegrated.scale(maxLength / length);
      }

      if (Math.abs(captureKi.getDoubleValue()) < 1e-10)
      {
         icpErrorIntegrated.set(0.0, 0.0);
      }

      icpErrorIntegrated.getFrameTuple2d(icpIntegral);
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

   private final FrameVector2d cmpError = new FrameVector2d();
   private final FrameVector2d cmpErrorPreviousValue = new FrameVector2d();
   private final FrameVector2d cmpErrorDifference = new FrameVector2d();

   private void rateLimitCMP(FramePoint2d cmp, FramePoint2d cmpPreviousValue, FramePoint2d perfectCMP, FramePoint2d previousPerfectCMP)
   {
      if (feedbackPartMaxRate.getDoubleValue() < 1.0e-3)
         return;

      cmpError.setToZero(cmp.getReferenceFrame());
      cmpError.sub(cmp, perfectCMP);

      cmpErrorPreviousValue.setToZero(cmp.getReferenceFrame());
      cmpErrorPreviousValue.sub(cmpPreviousValue, previousPerfectCMP);

      cmpErrorDifference.sub(cmpError, cmpErrorPreviousValue);
      double errorDifferenceMagnitude = cmpErrorDifference.length();
      double errorDifferenceMax = controlDT * feedbackPartMaxRate.getDoubleValue();
      if (errorDifferenceMagnitude > errorDifferenceMax)
         cmpErrorDifference.scale(errorDifferenceMax / errorDifferenceMagnitude);

      cmpError.add(cmpErrorPreviousValue, cmpErrorDifference);
      cmp.add(perfectCMP, cmpError);
   }

   private class Vector2dZUpFrame extends ReferenceFrame
   {
      private static final long serialVersionUID = -1810366869361449743L;
      private final FrameVector2d xAxis;
      private final Vector3D x = new Vector3D();
      private final Vector3D y = new Vector3D();
      private final Vector3D z = new Vector3D();
      private final RotationMatrix rotation = new RotationMatrix();

      public Vector2dZUpFrame(String string, ReferenceFrame parentFrame)
      {
         super(string, parentFrame);
         xAxis = new FrameVector2d(parentFrame);
      }

      public void setXAxis(FrameVector2d xAxis)
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
