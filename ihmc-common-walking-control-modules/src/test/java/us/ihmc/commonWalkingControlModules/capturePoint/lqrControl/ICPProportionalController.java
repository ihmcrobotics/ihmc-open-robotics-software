package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsProvider;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ICPProportionalController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FrameVector3D tempControl = new FrameVector3D(worldFrame);
   private final YoFrameVector3D dcmError = new YoFrameVector3D("dcmError", worldFrame, registry);
   private final YoFrameVector3D dcmErrorIntegrated = new YoFrameVector3D("dcmErrorIntegrated", "", worldFrame, registry);

   private final YoFrameVector3D feedbackPart = new YoFrameVector3D("dcmControlCMPFeedback", worldFrame, registry);
   private final YoFrameVector3D feedForwardPart = new YoFrameVector3D("dcmControlCMPFeedForward", worldFrame, registry);

   private final YoFramePoint3D cmpOutput = new YoFramePoint3D("dcmControlCMPOutput", worldFrame, registry);

   private final double controlDT;
   private final DoubleProvider captureKpParallelToMotion;
   private final DoubleProvider captureKpOrthogonalToMotion;

   private final DoubleProvider captureKi;
   private final DoubleProvider captureKiBleedoff;

   private final Vector2dZUpFrame icpVelocityDirectionFrame;

   private final FrameVector3D tempICPErrorIntegrated = new FrameVector3D(worldFrame);

   public ICPProportionalController(ICPControlGainsProvider gains, double controlDT, YoRegistry parentRegistry)
   {
      this.controlDT = controlDT;

      icpVelocityDirectionFrame = new Vector2dZUpFrame("icpVelocityDirectionFrame", worldFrame);

      parentRegistry.addChild(registry);

      captureKpParallelToMotion = gains.getYoKpParallelToMotion();
      captureKpOrthogonalToMotion = gains.getYoKpOrthogonalToMotion();
      captureKi = gains.getYoKi();
      captureKiBleedoff = gains.getYoIntegralLeakRatio();
   }

   public void reset()
   {
      dcmErrorIntegrated.setToZero();
   }

   public FramePoint3DReadOnly doProportionalControl(FramePoint3DReadOnly measuredDCMPosition, FramePoint3DReadOnly desiredDCMPosition,
                                                     FrameVector3DReadOnly desiredDCMVelocity, double omega0)
   {
      cmpOutput.setMatchingFrame(measuredDCMPosition);

      // feed forward part
      feedForwardPart.scaleAdd(-1.0 / omega0, desiredDCMVelocity, desiredDCMPosition);

      cmpOutput.sub(feedForwardPart);

      // feedback part
      dcmError.sub(measuredDCMPosition, desiredDCMPosition);

      tempControl.set(dcmError);
      double epsilonZeroICPVelocity = 1e-5;
      if (desiredDCMVelocity.lengthSquared() > MathTools.square(epsilonZeroICPVelocity))
      {
         icpVelocityDirectionFrame.setXAxis(tempControl);
         tempControl.changeFrame(icpVelocityDirectionFrame);
         tempControl.setX(tempControl.getX() * captureKpParallelToMotion.getValue());
         tempControl.setY(tempControl.getY() * captureKpOrthogonalToMotion.getValue());
         tempControl.changeFrame(cmpOutput.getReferenceFrame());
      }
      else
      {
         tempControl.scale(captureKpOrthogonalToMotion.getValue());
      }

      feedbackPart.set(tempControl);

      tempICPErrorIntegrated.set(dcmError);
      tempICPErrorIntegrated.scale(controlDT);
      tempICPErrorIntegrated.scale(captureKi.getValue());

      dcmErrorIntegrated.scale(captureKiBleedoff.getValue());
      dcmErrorIntegrated.add(tempICPErrorIntegrated);

      double length = dcmErrorIntegrated.length();
      double maxLength = 0.02;
      if (length > maxLength)
      {
         dcmErrorIntegrated.scale(maxLength / length);
      }

      if (Math.abs(captureKi.getValue()) < 1e-10)
      {
         dcmErrorIntegrated.setToZero();
      }

      feedbackPart.add(dcmErrorIntegrated);

      cmpOutput.add(feedForwardPart, feedbackPart);

      return cmpOutput;
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

      public void setXAxis(FrameTuple3DReadOnly xAxis)
      {
         this.xAxis.setIncludingFrame(xAxis);
         this.xAxis.changeFrame(getParent());
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
}
