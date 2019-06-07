package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.List;

public class QuadrupedSwingSpeedUpCalculator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final DoubleProvider omega;
   private final DoubleProvider controllerTime;

   private final BooleanProvider useSimpleSwingSpeedUpCalculation = new BooleanParameter("useSimpleSwingSpeedUpCalculation", registry, true);
   private final DoubleProvider durationForEmergencySwingSpeedUp = new DoubleParameter("durationForEmergencySwingSpeedUp", registry, 0.35);

   public QuadrupedSwingSpeedUpCalculator(QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      this(controllerToolbox.getLinearInvertedPendulumModel().getYoNaturalFrequency(), controllerToolbox.getRuntimeEnvironment().getRobotTimestamp(), parentRegistry);
   }

   public QuadrupedSwingSpeedUpCalculator(DoubleProvider omega, DoubleProvider controllerTime, YoVariableRegistry parentRegistry)
   {
      this.omega = omega;
      this.controllerTime = controllerTime;

      parentRegistry.addChild(registry);
   }

   public double estimateSwingSpeedUpTimeUnderDisturbance(List<? extends QuadrupedTimedStep> activeSteps, double normalizedEllipticalDCMError,
                                                          FramePoint3DReadOnly desiredICP, FramePoint3DReadOnly finalDesiredICP,
                                                          FramePoint3DReadOnly estimatedICP, FramePoint3DReadOnly perfectCMP)
   {
      if (normalizedEllipticalDCMError < 1.0)
         return 0.0;

      if (activeSteps.isEmpty())
         return 0.0;

      double deltaTimeToBeAccounted;
      if (useSimpleSwingSpeedUpCalculation.getValue())
      {
         double shortestSpeedUp = Double.POSITIVE_INFINITY;
         double currentTime = controllerTime.getValue();
         for (int i = 0; i < activeSteps.size(); i++)
         {
            QuadrupedTimedStep activeStep = activeSteps.get(i);
            double duration = activeStep.getTimeInterval().getDuration();
            double phaseThroughStep = (currentTime - activeStep.getTimeInterval().getStartTime()) / duration;
            double nominalRemainingTime = durationForEmergencySwingSpeedUp.getValue() * (1.0 - phaseThroughStep);
            double actualRemainingTime = duration - nominalRemainingTime;

            shortestSpeedUp = Math.min(shortestSpeedUp, actualRemainingTime - nominalRemainingTime);
         }

         deltaTimeToBeAccounted = Math.max(shortestSpeedUp, 0.0);
      }
      else
      {
         deltaTimeToBeAccounted = estimateDeltaTimeBetweenDesiredICPAndActualICP(desiredICP, finalDesiredICP, estimatedICP, perfectCMP);
      }

      if (Double.isNaN(deltaTimeToBeAccounted))
         return 0.0;

      return deltaTimeToBeAccounted;
   }

   private final FrameLine2D desiredICPToFinalICPLine = new FrameLine2D();
   private final FrameLineSegment2D desiredICPToFinalICPLineSegment = new FrameLineSegment2D();

   private final FrameVector2D icpError = new FrameVector2D();
   private final FrameLine2D adjustedICPDynamicsLine = new FrameLine2D();
   private final FramePoint2D offsetFinalDesiredICP = new FramePoint2D();
   private final FramePoint2D projectedDesiredICP = new FramePoint2D();
   private final FramePoint2D projectedEstimatedICP = new FramePoint2D();

   private final FramePoint2D desiredICP = new FramePoint2D();
   private final FramePoint2D finalDesiredICP = new FramePoint2D();
   private final FramePoint2D estimatedICP = new FramePoint2D();

   private double estimateDeltaTimeBetweenDesiredICPAndActualICP(FramePoint3DReadOnly desiredDCM, FramePoint3DReadOnly finalDesiredDCM,
                                                                 FramePoint3DReadOnly estimatedDCM, FramePoint3DReadOnly perfectCMP)
   {
      perfectCMP.checkReferenceFrameMatch(worldFrame);

      desiredICP.set(desiredDCM);
      finalDesiredICP.set(finalDesiredDCM);
      estimatedICP.set(estimatedDCM);

      /*
       * The ICP plan is not being updated with the step adjustment. We approximate the step adjustment here by offsetting the final ICP and then projecting
       * the desired ICP onto these dynamics. Without this, the foot will not be set down more quickly for lateral errors
       */
      icpError.sub(estimatedICP, desiredICP);

      double estimatedDesiredExponential = perfectCMP.distanceXY(finalDesiredICP) / perfectCMP.distanceXY(desiredICP);
      offsetFinalDesiredICP.scaleAdd(estimatedDesiredExponential, icpError, finalDesiredICP);

      if (estimatedICP.distance(offsetFinalDesiredICP) < 1.0e-10)
         return Double.NaN;

      projectedDesiredICP.setIncludingFrame(desiredICP);
      adjustedICPDynamicsLine.set(estimatedICP, offsetFinalDesiredICP);
      adjustedICPDynamicsLine.orthogonalProjection(projectedDesiredICP); // projects the desired icp onto this dynamics line

      if (projectedDesiredICP.distance(offsetFinalDesiredICP) < 1.0e-10)
      {
         return Double.NaN;
      }

      desiredICPToFinalICPLineSegment.set(projectedDesiredICP, offsetFinalDesiredICP);
      double percentAlongLineSegmentICP = desiredICPToFinalICPLineSegment.percentageAlongLineSegment(estimatedICP);
      if (percentAlongLineSegmentICP < 0.0)
      {
         desiredICPToFinalICPLine.set(projectedDesiredICP, offsetFinalDesiredICP);
         projectedDesiredICP.setIncludingFrame(estimatedDCM);
         desiredICPToFinalICPLine.orthogonalProjection(projectedEstimatedICP);
      }

      double actualDistanceDueToDisturbance = perfectCMP.distanceXY(projectedEstimatedICP);
      double expectedDistanceAccordingToPlan = perfectCMP.distanceXY(offsetFinalDesiredICP);

      double distanceRatio = actualDistanceDueToDisturbance / expectedDistanceAccordingToPlan;

      if (distanceRatio < 1.0e-3)
         return 0.0;
      else
         return Math.log(distanceRatio) / omega.getValue();
   }
}
