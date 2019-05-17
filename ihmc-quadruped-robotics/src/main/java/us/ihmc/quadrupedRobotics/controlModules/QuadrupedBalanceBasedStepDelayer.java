package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.util.ArrayList;
import java.util.List;

public class QuadrupedBalanceBasedStepDelayer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final QuadrantDependentList<? extends PlaneContactState> contactStates;

   private final FrameVector2D icpError = new FrameVector2D();
   private final QuadrupedReferenceFrames referenceFrames;

   private final QuadrantDependentList<YoBoolean> areFeetDelayed = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFramePoint3D> delayedFootLocations = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoDouble> delayDurations = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoDouble> timeScaledEllipticalError = new QuadrantDependentList<>();

   private final List<QuadrupedTimedStep> stepsStarting = new ArrayList<>();

   private final FramePoint3D tempFootPoint3D = new FramePoint3D();
   private final FramePoint2D tempFootPoint2D = new FramePoint2D();
   private final FramePoint3D currentDCM = new FramePoint3D();
   private final FramePoint2D currentICP = new FramePoint2D();

   private final BooleanProvider allowDelayingSteps = new BooleanParameter("allowingDelayingSteps", registry, true);
   private final BooleanProvider delayAllSubsequentSteps = new BooleanParameter("delayAllSubsequentSteps", registry, true);
   private final BooleanProvider requireTwoFeetInContact = new BooleanParameter("requireTwoFeetInContact", registry, true);

   private final DoubleProvider maximumDelayDuration = new DoubleParameter("maximumDelayDuration", registry, 0.1);
   private final IntegerProvider controlTicksToDelay = new IntegerParameter("controlTicksToDelay", registry, 1);
   private final YoBoolean aboutToHaveNoLeftFoot = new YoBoolean("aboutToHaveNoLeftFoot", registry);
   private final YoBoolean aboutToHaveNoRightFoot = new YoBoolean("aboutToHaveNoRightFoot", registry);

   private final DoubleProvider timeScaledEllipticalErrorThreshold = new DoubleParameter("timeScaledEllipticalErrorThreshold", registry, 5.0);
   private final DoubleProvider thresholdScalerForNoFeetOnSide = new DoubleParameter("thresholdScalerForNoFeetOnSide", registry, 4.0);

   private final DoubleProvider omega;
   private final ReferenceFrame midZUpFrame;
   private final QuadrupedControllerToolbox controllerToolbox;
   private final double controlDt;

   public QuadrupedBalanceBasedStepDelayer(QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      referenceFrames = controllerToolbox.getReferenceFrames();

      omega = controllerToolbox.getLinearInvertedPendulumModel().getYoNaturalFrequency();

      midZUpFrame = controllerToolbox.getSupportPolygons().getMidFeetZUpFrame();
      contactStates = controllerToolbox.getFootContactStates();

      controlDt = controllerToolbox.getRuntimeEnvironment().getControlDT();

      YoGraphicsListRegistry graphicsListRegistry = controllerToolbox.getRuntimeEnvironment().getGraphicsListRegistry();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         areFeetDelayed.put(robotQuadrant, new YoBoolean(robotQuadrant.getCamelCaseName() + "_IsFootDelayed", registry));
         delayDurations.put(robotQuadrant, new YoDouble(robotQuadrant.getCamelCaseName() + "_DelayDuration", registry));
         timeScaledEllipticalError.put(robotQuadrant, new YoDouble(robotQuadrant.getCamelCaseName() + "TimeScaledEllipticalError", registry));

         String name = robotQuadrant.getCamelCaseName() + "_DelayedStepLocation";
         YoFramePoint3D delayedFootLocation = new YoFramePoint3D(name, worldFrame, registry);
         delayedFootLocation.setToNaN();
         delayedFootLocations.put(robotQuadrant, delayedFootLocation);

         graphicsListRegistry.registerYoGraphic("StepDelayer", new YoGraphicPosition(name + "Vis", delayedFootLocation, 0.05, YoAppearance.Orange()));
      }

      parentRegistry.addChild(registry);
   }

   private final List<QuadrupedTimedStep> inactiveSteps = new ArrayList<>();

   public boolean delayStepsIfNecessary(List<? extends QuadrupedTimedStep> activeSteps, List<? extends QuadrupedTimedStep> allOtherSteps,
                                        FramePoint3DReadOnly desiredDcm, double normalizedDcmEllipticalError)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         delayedFootLocations.get(robotQuadrant).setToNaN();

      if (!allowDelayingSteps.getValue())
      {
         return false;
      }

      controllerToolbox.getDCMPositionEstimate(currentDCM);
      currentICP.setIncludingFrame(currentDCM);
      icpError.setIncludingFrame(currentICP);
      icpError.sub(desiredDcm.getX(), desiredDcm.getY());

      int numberOfLeftSideFeetInContact = 0;
      int numberOfRightSideFeetInContact = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (!contactStates.get(robotQuadrant).inContact())
            continue;

         if (robotQuadrant.isQuadrantOnLeftSide())
            numberOfLeftSideFeetInContact++;
         else
            numberOfRightSideFeetInContact++;
      }

      inactiveSteps.clear();
      for (int i = 0; i < allOtherSteps.size(); i++)
         inactiveSteps.add(allOtherSteps.get(i));

      stepsStarting.clear();

      boolean leftSideHasStartingStep = false;
      boolean rightSideHasStartingStep = false;

      for (int i = 0; i < activeSteps.size(); i++)
      {
         QuadrupedTimedStep activeStep = activeSteps.get(i);
         inactiveSteps.remove(activeStep);

         if (contactStates.get(activeStep.getRobotQuadrant()).inContact())
         {
            if (activeStep.getRobotQuadrant().isQuadrantOnLeftSide())
               leftSideHasStartingStep = true;
            else
               rightSideHasStartingStep = true;
            stepsStarting.add(activeStep);
         }
      }

      aboutToHaveNoLeftFoot.set(leftSideHasStartingStep && numberOfLeftSideFeetInContact == 1);
      aboutToHaveNoRightFoot.set(rightSideHasStartingStep && numberOfRightSideFeetInContact == 1);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         areFeetDelayed.get(robotQuadrant).set(false);
         if (!contactStates.get(robotQuadrant).inContact())
            delayDurations.get(robotQuadrant).set(0.0);
      }


      icpError.changeFrameAndProjectToXYPlane(midZUpFrame);

      double delayAmount = controlTicksToDelay.getValue() * controlDt;
      boolean stepWasDelayed = false;
      
      boolean forceDelay = (numberOfLeftSideFeetInContact + numberOfRightSideFeetInContact - stepsStarting.size() < 2) && requireTwoFeetInContact.getValue();

      for (int i = 0; i < stepsStarting.size(); i++)
      {
         QuadrupedTimedStep stepStarting = stepsStarting.get(i);
         RobotQuadrant quadrantStarting = stepStarting.getRobotQuadrant();

         double scaledNormalizedDCMEllipticalError = Math.exp(omega.getValue() * stepStarting.getTimeInterval().getDuration()) * normalizedDcmEllipticalError;

         icpError.changeFrameAndProjectToXYPlane(midZUpFrame);

         if (icpError.getY() > 0.0 && stepStarting.getRobotQuadrant().isQuadrantOnLeftSide() && aboutToHaveNoLeftFoot.getBooleanValue())
            scaledNormalizedDCMEllipticalError *= thresholdScalerForNoFeetOnSide.getValue();
         else if (icpError.getY() < 0.0 && stepStarting.getRobotQuadrant().isQuadrantOnRightSide() && aboutToHaveNoRightFoot.getBooleanValue())
            scaledNormalizedDCMEllipticalError *= thresholdScalerForNoFeetOnSide.getValue();

         timeScaledEllipticalError.get(quadrantStarting).set(scaledNormalizedDCMEllipticalError);
         if (!forceDelay && scaledNormalizedDCMEllipticalError < timeScaledEllipticalErrorThreshold.getValue())
            continue;

         icpError.changeFrameAndProjectToXYPlane(worldFrame);

         if (isFootPushingAgainstError(quadrantStarting, currentICP) || forceDelay)
         {
            YoDouble delayDuration = delayDurations.get(quadrantStarting);

            if (delayDuration.getDoubleValue() + delayAmount < maximumDelayDuration.getValue())
            {
               TimeIntervalBasics timeInterval = stepStarting.getTimeInterval();
               double currentStartTime = timeInterval.getStartTime();
               if (delayAllSubsequentSteps.getValue())
                  stepStarting.getTimeInterval().shiftInterval(delayAmount);
               else
                  stepStarting.getTimeInterval().setStartTime(currentStartTime + delayAmount);
               delayDuration.add(delayAmount);
               areFeetDelayed.get(quadrantStarting).set(true);
               stepWasDelayed = true;
            }
         }
      }

      if (stepWasDelayed && delayAllSubsequentSteps.getValue())
      {
         for (int i = 0; i < inactiveSteps.size(); i++)
            inactiveSteps.get(i).getTimeInterval().shiftInterval(delayAmount);
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FixedFramePoint3DBasics delayedFootLocation = delayedFootLocations.get(robotQuadrant);
         if (areFeetDelayed.get(robotQuadrant).getBooleanValue())
         {
            tempFootPoint3D.setToZero(referenceFrames.getSoleFrame(robotQuadrant));
            tempFootPoint3D.changeFrame(worldFrame);
            delayedFootLocation.set(tempFootPoint3D);
         }
         else
         {
            delayedFootLocation.setToNaN();
         }
      }

      return stepWasDelayed;
   }

   private final FramePoint3D tempOtherFootPoint = new FramePoint3D();
   private final FramePoint2D tempOtherFootPoint2D = new FramePoint2D();
   private final FramePoint2D intersectionToThrowAway = new FramePoint2D();

   private boolean isFootPushingAgainstError(RobotQuadrant robotQuadrant, FramePoint2DReadOnly currentDcmPosition)
   {
      RobotQuadrant otherSideToCheck = RobotQuadrant.getQuadrant(robotQuadrant.getEnd(), robotQuadrant.getOppositeSide());
      RobotQuadrant otherEndToCheck = RobotQuadrant.getQuadrant(robotQuadrant.getOppositeEnd(), robotQuadrant.getOppositeSide());

      tempFootPoint3D.setToZero(referenceFrames.getSoleFrame(robotQuadrant));
      tempFootPoint3D.changeFrame(worldFrame);
      tempFootPoint2D.setIncludingFrame(tempFootPoint3D);

      if (contactStates.get(otherEndToCheck).inContact())
      {
         tempOtherFootPoint.setToZero(referenceFrames.getSoleFrame(otherEndToCheck));
         tempOtherFootPoint.changeFrame(worldFrame);
         tempOtherFootPoint2D.setIncludingFrame(tempOtherFootPoint);

         if (EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(currentDcmPosition, icpError, tempFootPoint2D, tempOtherFootPoint2D, intersectionToThrowAway))
            return true;
      }

      if (contactStates.get(otherSideToCheck).inContact())
      {
         tempOtherFootPoint.setToZero(referenceFrames.getSoleFrame(otherSideToCheck));
         tempOtherFootPoint.changeFrame(worldFrame);
         tempOtherFootPoint2D.setIncludingFrame(tempOtherFootPoint);

         if (EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(currentDcmPosition, icpError, tempFootPoint2D, tempOtherFootPoint2D, intersectionToThrowAway))
            return true;
      }

      return false;
   }
}
