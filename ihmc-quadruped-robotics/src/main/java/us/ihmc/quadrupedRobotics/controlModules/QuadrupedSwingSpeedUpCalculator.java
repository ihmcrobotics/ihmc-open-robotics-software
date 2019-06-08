package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalBasics;
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
   private final BooleanProvider speedUpIfItWillHelpButIsntNeeded = new BooleanParameter("speedUpIfItWillHelpButIsntNeeded", registry, false);
   private final DoubleProvider durationForEmergencySwingSpeedUp = new DoubleParameter("durationForEmergencySwingSpeedUp", registry, 0.35);

   private final DoubleProvider minDistanceInsideForSpeedUp = new DoubleParameter("minDistanceInsideForSpeedUp", registry, 0.0);

   private final FramePoint2D tempFootPoint2D = new FramePoint2D();
   private final FramePoint2D goalPosition = new FramePoint2D();

   private final QuadrantDependentList<MovingReferenceFrame> soleFrames;
   private final QuadrantDependentList<? extends PlaneContactState> contactStates;
   private final FrameConvexPolygon2D supportPolygonInWorld = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D supportPolygonInWorldAfterChange = new FrameConvexPolygon2D();


   public QuadrupedSwingSpeedUpCalculator(QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      this(controllerToolbox.getReferenceFrames().getSoleFrames(), controllerToolbox.getFootContactStates(),
           controllerToolbox.getLinearInvertedPendulumModel().getYoNaturalFrequency(),
           controllerToolbox.getRuntimeEnvironment().getRobotTimestamp(), parentRegistry);
   }

   public QuadrupedSwingSpeedUpCalculator(QuadrantDependentList<MovingReferenceFrame> soleFrames,
                                          QuadrantDependentList<? extends PlaneContactState> contactStates, DoubleProvider omega, DoubleProvider controllerTime,
                                          YoVariableRegistry parentRegistry)
   {
      this.soleFrames = soleFrames;
      this.contactStates = contactStates;
      this.omega = omega;
      this.controllerTime = controllerTime;

      parentRegistry.addChild(registry);
   }

   private final FramePoint2D desiredICP = new FramePoint2D();
   private final FramePoint2D finalDesiredICP = new FramePoint2D();
   private final FramePoint2D estimatedICP = new FramePoint2D();

   public double estimateSwingSpeedUpTimeUnderDisturbance(List<? extends QuadrupedTimedStep> activeSteps, double normalizedEllipticalDCMError,
                                                          FramePoint3DReadOnly desiredDCM, FramePoint3DReadOnly finalDesiredDCM,
                                                          FramePoint3DReadOnly estimatedDCM, FramePoint3DReadOnly perfectCMP)
   {
      if (normalizedEllipticalDCMError < 1.0)
         return 0.0;

      if (activeSteps.isEmpty())
         return 0.0;

      desiredICP.set(desiredDCM);
      finalDesiredICP.set(finalDesiredDCM);
      estimatedICP.set(estimatedDCM);

      double deltaTimeToBeAccounted;
      if (useSimpleSwingSpeedUpCalculation.getValue())
      {
         double shortestSpeedUp = Double.POSITIVE_INFINITY;
         double currentTime = controllerTime.getValue();
         for (int i = 0; i < activeSteps.size(); i++)
         {
            QuadrupedTimedStep activeStep = activeSteps.get(i);
            goalPosition.setIncludingFrame(worldFrame, activeStep.getGoalPosition());

            if (!wouldPuttingTheFootDownHelpWithErrorRejection(activeStep.getRobotQuadrant(), goalPosition))
               continue;

            TimeIntervalBasics timeInterval = activeStep.getTimeInterval();
            double duration = timeInterval.getDuration();
            double phaseThroughStep = (currentTime - timeInterval.getStartTime()) / duration;
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


   private boolean wouldPuttingTheFootDownHelpWithErrorRejection(RobotQuadrant robotQuadrant, FramePoint2DReadOnly goalPosition)
   {
      updateSupportPolygon(supportPolygonInWorld, contactStates, null, null);
      updateSupportPolygon(supportPolygonInWorldAfterChange, contactStates, robotQuadrant, goalPosition);

      boolean currentICPFarEnoughInsideSupport = supportPolygonInWorld.signedDistance(estimatedICP) < -minDistanceInsideForSpeedUp.getValue();

      if (willPuttingTheFootDownMakeTrackingPossibleAgain(currentICPFarEnoughInsideSupport))
         return true;

      if (currentICPFarEnoughInsideSupport || supportPolygonInWorld.isPointInside(desiredICP))
      {
         if (speedUpIfItWillHelpButIsntNeeded.getValue())
            return willPuttingTheFootDownHelpTracking();
         else
            return false;
      }
      else
      {
         // both the desired and the current values are outside the support polygon
         return willPuttingTheFootDownHelpTracking();
      }
   }

   private boolean willPuttingTheFootDownMakeTrackingPossibleAgain(boolean currentICPFarEnoughInsideSupport)
   {
      boolean trackingIsCurrentlyImpossible = supportPolygonInWorld.isPointInside(desiredICP) && !currentICPFarEnoughInsideSupport;
      boolean trackingWillBePossible = supportPolygonInWorldAfterChange.isPointInside(desiredICP) && supportPolygonInWorldAfterChange.isPointInside(estimatedICP);

      return trackingIsCurrentlyImpossible && trackingWillBePossible;
   }

   private boolean willPuttingTheFootDownHelpTracking()
   {
      double currentDesiredDistance = supportPolygonInWorld.signedDistance(desiredICP);
      double currentEstimatedDistance = supportPolygonInWorld.signedDistance(estimatedICP);

      double desiredDistanceAfterChange = supportPolygonInWorldAfterChange.signedDistance(desiredICP);
      double estimatedDistanceAfterChange = supportPolygonInWorldAfterChange.signedDistance(estimatedICP);

      return (desiredDistanceAfterChange < currentDesiredDistance) && (estimatedDistanceAfterChange < currentEstimatedDistance);
   }

   private final FrameVector2D icpError = new FrameVector2D();
   private final FrameLine2D adjustedICPDynamicsLine = new FrameLine2D();
   private final FramePoint2D offsetFinalDesiredICP = new FramePoint2D();
   private final FramePoint2D projectedDesiredICP = new FramePoint2D();
   private final FramePoint2D projectedEstimatedICP = new FramePoint2D();

   private double estimateDeltaTimeBetweenDesiredICPAndActualICP(FramePoint2DReadOnly desiredICP, FramePoint2DReadOnly finalDesiredICP,
                                                                 FramePoint2DReadOnly estimatedICP, FramePoint3DReadOnly perfectCMP)
   {
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

      projectEstimatedICPOntoDynamics(projectedEstimatedICP, estimatedICP, desiredICP, finalDesiredICP);

      double actualDistanceDueToDisturbance = perfectCMP.distanceXY(projectedEstimatedICP);
      double expectedDistanceAccordingToPlan = perfectCMP.distanceXY(offsetFinalDesiredICP);

      double distanceRatio = actualDistanceDueToDisturbance / expectedDistanceAccordingToPlan;

      if (distanceRatio < 1.0e-3)
         return 0.0;
      else
         return Math.log(distanceRatio) / omega.getValue();
   }

   private final FrameLineSegment2D desiredICPToFinalICPLineSegment = new FrameLineSegment2D();

   private double projectEstimatedICPOntoDynamics(FixedFramePoint2DBasics projectedEstimatedICPToPack, FramePoint2DReadOnly estimatedICP,
                                                  FramePoint2DReadOnly desiredICP, FramePoint2DReadOnly finalDesiredICP)
   {
      if (desiredICP.distance(finalDesiredICP) < 1e-5)
      {
         projectedEstimatedICPToPack.set(estimatedICP);
         return desiredICP.distance(estimatedICP);
      }

      desiredICPToFinalICPLineSegment.set(desiredICP, finalDesiredICP);
      desiredICPToFinalICPLineSegment.orthogonalProjection(estimatedICP, projectedEstimatedICPToPack);

      return desiredICP.distance(projectedEstimatedICPToPack);
   }



   private void updateSupportPolygon(FrameConvexPolygon2DBasics polygonToPack, QuadrantDependentList<? extends PlaneContactState> contactStates,
                                     RobotQuadrant quadrantToAdd, FramePoint2DReadOnly positionToAdd)
   {
      polygonToPack.clear();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (!contactStates.get(robotQuadrant).inContact())
         {
            if (quadrantToAdd == robotQuadrant)
               polygonToPack.addVertex(positionToAdd);
            else
               continue;
         }

         tempFootPoint2D.setToZero(soleFrames.get(robotQuadrant));
         tempFootPoint2D.changeFrameAndProjectToXYPlane(worldFrame);
         polygonToPack.addVertex(tempFootPoint2D);
      }
      polygonToPack.update();
   }

}
