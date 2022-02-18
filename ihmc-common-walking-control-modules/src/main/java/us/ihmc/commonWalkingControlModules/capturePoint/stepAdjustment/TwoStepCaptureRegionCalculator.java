package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.awt.*;

public class TwoStepCaptureRegionCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FrameConvexPolygon2D twoStepRegion = new FrameConvexPolygon2D();

   private final YoBoolean hasTwoStepRegion = new YoBoolean("hasTwoStepRegion", registry);
   private final YoFrameConvexPolygon2D yoTwoStepRegion = new YoFrameConvexPolygon2D("twoStepCaptureRegion", ReferenceFrame.getWorldFrame(), 30, registry);

   public TwoStepCaptureRegionCalculator(YoRegistry parentRegistry,
                                         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon polygonArtifact = new YoArtifactPolygon("Two Step Capture Region", yoTwoStepRegion, Color.GREEN, false);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), polygonArtifact);
      }

      parentRegistry.addChild(registry);
   }

   private final FramePoint2D inversePoint = new FramePoint2D();
   private final FramePoint2D inverseGoal = new FramePoint2D();
   private final FramePoint2D stepPosition = new FramePoint2D();
   private final FramePoint2D touchdownPoint = new FramePoint2D();

   public void reset()
   {
      hasTwoStepRegion.set(false);
      yoTwoStepRegion.clear();
   }

   public void computeFromStepGoal(double minSwingAndTransferTime, SimpleFootstep stepGoal, double omega, FrameConvexPolygon2DReadOnly oneStepCaptureRegion)
   {
      hasTwoStepRegion.set(true);
      twoStepRegion.clear();
      twoStepRegion.setMatchingFrame(oneStepCaptureRegion, false);
      twoStepRegion.changeFrame(ReferenceFrame.getWorldFrame());

      stepPosition.set(stepGoal.getSoleFramePose().getPosition());

      double minExponential = Math.exp(-omega * minSwingAndTransferTime);
      double radical = 1.0 / (1.0 - minExponential);

      inverseGoal.setAndScale(minExponential, stepPosition);

      for (int i = 0; i < oneStepCaptureRegion.getNumberOfVertices(); i++)
      {
         touchdownPoint.setIncludingFrame(oneStepCaptureRegion.getVertex(i));
         touchdownPoint.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
         inversePoint.sub(touchdownPoint, inverseGoal);
         inversePoint.scale(radical);

         twoStepRegion.addVertex(inversePoint);
      }

      twoStepRegion.update();
      yoTwoStepRegion.set(twoStepRegion);
   }

   public FrameConvexPolygon2DReadOnly getCaptureRegion()
   {
      return yoTwoStepRegion;
   }

   public boolean hasTwoStepRegion()
   {
      return hasTwoStepRegion.getBooleanValue();
   }
}
