package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsProvider;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class ICPProportionalController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FrameVector3D tempControl = new FrameVector3D(worldFrame);
   private final YoFrameVector3D icpError = new YoFrameVector3D("icpError", "", worldFrame, registry);
   private final YoFrameVector3D icpErrorIntegrated = new YoFrameVector3D("icpErrorIntegrated", "", worldFrame, registry);

   private final YoFrameVector3D feedbackPart = new YoFrameVector3D("feedbackPart", worldFrame, registry);

   private final YoFramePoint3D cmpOutput = new YoFramePoint3D("icpControlCMPOutput", worldFrame, registry);

   private final FramePoint3D icpPosition = new FramePoint3D();

   private final double controlDT;
   private final DoubleProvider captureKpParallelToMotion;
   private final DoubleProvider captureKpOrthogonalToMotion;

   private final DoubleProvider captureKi;
   private final DoubleProvider captureKiBleedoff;

   private final Vector2dZUpFrame icpVelocityDirectionFrame;

   private final FrameVector3D tempICPErrorIntegrated = new FrameVector3D(worldFrame);

   public ICPProportionalController(ICPControlGainsProvider gains, double controlDT, YoVariableRegistry parentRegistry)
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
      icpErrorIntegrated.setToZero();
   }

   public FramePoint3DReadOnly doProportionalControl(FramePoint3DReadOnly capturePoint, FramePoint3DReadOnly desiredCapturePoint,
                                                     FrameVector3DReadOnly desiredCapturePointVelocity, double omega0)
   {
      cmpOutput.setMatchingFrame(capturePoint);

      icpPosition.setIncludingFrame(capturePoint);
      icpPosition.changeFrame(worldFrame);
      // feed forward part
      tempControl.setIncludingFrame(desiredCapturePointVelocity);
      tempControl.scale(1.0 / omega0);
      tempControl.changeFrame(worldFrame);

      cmpOutput.sub(tempControl);

      // feedback part
      icpError.set(icpPosition);
      icpError.sub(desiredCapturePoint);

      tempControl.set(icpError);
      double epsilonZeroICPVelocity = 1e-5;
      if (desiredCapturePointVelocity.lengthSquared() > MathTools.square(epsilonZeroICPVelocity))
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
         icpErrorIntegrated.setToZero();
      }

      tempControl.add(icpErrorIntegrated);

      feedbackPart.set(tempControl);
      cmpOutput.add(tempControl);

      return cmpOutput;
   }

   private class Vector2dZUpFrame extends ReferenceFrame
   {
      private final FrameVector3D xAxis;
      private final Vector3D x = new Vector3D();
      private final Vector3D y = new Vector3D();
      private final Vector3D z = new Vector3D();
      private final RotationMatrix rotation = new RotationMatrix();

      public Vector2dZUpFrame(String string, ReferenceFrame parentFrame)
      {
         super(string, parentFrame);
         xAxis = new FrameVector3D(parentFrame);
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
