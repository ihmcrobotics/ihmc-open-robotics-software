package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.QuadrupedSupportPolygons;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
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
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
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
   private final List<QuadrupedTimedStep> stepsAlreadyActive = new ArrayList<>();
   private final List<QuadrupedTimedStep> updatedActiveSteps = new ArrayList<>();

   private final FramePoint3D tempFootPoint3D = new FramePoint3D();
   private final FramePoint2D tempFootPoint2D = new FramePoint2D();
   private final FramePoint3D currentDCM = new FramePoint3D();
   private final FramePoint2D currentICP = new FramePoint2D();
   private final FramePoint2D desiredICP = new FramePoint2D();

   private final BooleanProvider allowDelayingSteps = new BooleanParameter("allowingDelayingSteps", registry, true);
   private final BooleanProvider delayAllSubsequentSteps = new BooleanParameter("delayAllSubsequentSteps", registry, true);
   private final BooleanProvider requireTwoFeetInContact = new BooleanParameter("requireTwoFeetInContact", registry, true);

   private final DoubleProvider maximumDelayFraction = new DoubleParameter("maximumDelayFraction", registry, 5.0);
   private final DoubleParameter timeToDelayLiftOff = new DoubleParameter("timeToDelayLiftOff", registry, 0.05);
   private final YoBoolean aboutToHaveNoLeftFoot = new YoBoolean("aboutToHaveNoLeftFoot", registry);
   private final YoBoolean aboutToHaveNoRightFoot = new YoBoolean("aboutToHaveNoRightFoot", registry);

   private final DoubleProvider timeScaledEllipticalErrorThreshold = new DoubleParameter("timeScaledEllipticalErrorThreshold", registry, 5.0);
   private final DoubleProvider thresholdScalerForNoFeetOnSide = new DoubleParameter("thresholdScalerForNoFeetOnSide", registry, 4.0);

   private final DoubleProvider omega;
   private final ReferenceFrame midZUpFrame;
   private final QuadrupedControllerToolbox controllerToolbox;

   private final FrameConvexPolygon2DReadOnly supportPolygonInWorld;
   private final FrameConvexPolygon2D supportPolygonInWorldAfterChange = new FrameConvexPolygon2D();

   public QuadrupedBalanceBasedStepDelayer(QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      referenceFrames = controllerToolbox.getReferenceFrames();
      supportPolygonInWorld = controllerToolbox.getSupportPolygons().getSupportPolygonInWorld();

      omega = controllerToolbox.getLinearInvertedPendulumModel().getYoNaturalFrequency();

      midZUpFrame = controllerToolbox.getSupportPolygons().getMidFeetZUpFrame();
      contactStates = controllerToolbox.getFootContactStates();

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

   public List<? extends QuadrupedTimedStep> delayStepsIfNecessary(List<? extends QuadrupedTimedStep> activeSteps,
                                                                   List<? extends QuadrupedTimedStep> allOtherSteps, FramePoint3DReadOnly desiredDcmPosition,
                                                                   double normalizedDcmEllipticalError)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         delayedFootLocations.get(robotQuadrant).setToNaN();

      if (!allowDelayingSteps.getValue())
      {
         return activeSteps;
      }

      stepsStarting.clear();
      inactiveSteps.clear();
      stepsAlreadyActive.clear();
      updatedActiveSteps.clear();

      controllerToolbox.getDCMPositionEstimate(currentDCM);
      currentICP.setIncludingFrame(currentDCM);
      desiredICP.setIncludingFrame(desiredDcmPosition);
      icpError.setIncludingFrame(currentICP);
      icpError.checkReferenceFrameMatch(desiredDcmPosition);
      icpError.sub(desiredDcmPosition.getX(), desiredDcmPosition.getY());

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

      for (int i = 0; i < allOtherSteps.size(); i++)
         inactiveSteps.add(allOtherSteps.get(i));

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
         else
         {
            stepsAlreadyActive.add(activeStep);
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

      double delayAmount = timeToDelayLiftOff.getValue();
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
         {
            updatedActiveSteps.add(stepStarting);
            continue;
         }

         icpError.changeFrameAndProjectToXYPlane(worldFrame);

         YoDouble delayDuration = delayDurations.get(quadrantStarting);
         boolean delayStep = isFootPushingAgainstError(quadrantStarting) || forceDelay;
         delayStep &= delayDuration.getDoubleValue() + delayAmount < (maximumDelayFraction.getValue() * stepStarting.getTimeInterval().getDuration());

         if (delayStep)
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
         else
         {
            updatedActiveSteps.add(stepStarting);
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

      for (int i = 0; i < stepsAlreadyActive.size(); i++)
         updatedActiveSteps.add(stepsAlreadyActive.get(i));

      return updatedActiveSteps;
   }

   private final FramePoint3D tempOtherFootPoint = new FramePoint3D();
   private final FramePoint2D tempOtherFootPoint2D = new FramePoint2D();
   private final FramePoint2D intersectionToThrowAway = new FramePoint2D();

   private boolean isFootPushingAgainstError(RobotQuadrant robotQuadrant)
   {
      boolean statesAreCapturable = supportPolygonInWorld.isPointInside(currentICP) || supportPolygonInWorld.isPointInside(desiredICP);
      if (statesAreCapturable)
         return isFootPushingAgainstErrorWhenTheDesiredsAreCapturable(robotQuadrant);
      else
         return isFootPushingAgainstErrorWhenTheDesiredsAreNotCapturable(robotQuadrant);
   }

   private boolean isFootPushingAgainstErrorWhenTheDesiredsAreCapturable(RobotQuadrant quadrantToBePickedUp)
   {
      RobotQuadrant otherSideToCheck = RobotQuadrant.getQuadrant(quadrantToBePickedUp.getEnd(), quadrantToBePickedUp.getOppositeSide());
      RobotQuadrant otherEndToCheck = RobotQuadrant.getQuadrant(quadrantToBePickedUp.getOppositeEnd(), quadrantToBePickedUp.getOppositeSide());

      tempFootPoint3D.setToZero(referenceFrames.getSoleFrame(quadrantToBePickedUp));
      tempFootPoint3D.changeFrame(worldFrame);
      tempFootPoint2D.setIncludingFrame(tempFootPoint3D);

      if (contactStates.get(otherEndToCheck).inContact())
      {
         tempOtherFootPoint.setToZero(referenceFrames.getSoleFrame(otherEndToCheck));
         tempOtherFootPoint.changeFrame(worldFrame);
         tempOtherFootPoint2D.setIncludingFrame(tempOtherFootPoint);

         if (EuclidGeometryTools
               .intersectionBetweenRay2DAndLineSegment2D(currentICP, icpError, tempFootPoint2D, tempOtherFootPoint2D, intersectionToThrowAway))
            return true;
      }

      if (contactStates.get(otherSideToCheck).inContact())
      {
         tempOtherFootPoint.setToZero(referenceFrames.getSoleFrame(otherSideToCheck));
         tempOtherFootPoint.changeFrame(worldFrame);
         tempOtherFootPoint2D.setIncludingFrame(tempOtherFootPoint);

         if (EuclidGeometryTools
               .intersectionBetweenRay2DAndLineSegment2D(currentICP, icpError, tempFootPoint2D, tempOtherFootPoint2D, intersectionToThrowAway))
            return true;
      }

      supportPolygonInWorldAfterChange.clear();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (quadrantToBePickedUp == robotQuadrant || !contactStates.get(robotQuadrant).inContact())
            continue;

         tempFootPoint2D.setToZero(referenceFrames.getSoleFrame(robotQuadrant));
         tempFootPoint2D.changeFrameAndProjectToXYPlane(worldFrame);
         supportPolygonInWorldAfterChange.addVertex(tempFootPoint2D);
      }
      supportPolygonInWorldAfterChange.update();

      if (supportPolygonInWorldAfterChange.isPointInside(desiredICP) && !supportPolygonInWorldAfterChange.isPointInside(currentICP))
         return true;

      return false;
   }

   private boolean isFootPushingAgainstErrorWhenTheDesiredsAreNotCapturable(RobotQuadrant robotQuadrant)
   {
      icpError.changeFrameAndProjectToXYPlane(midZUpFrame);

      if (icpError.getY() > 0.0 && robotQuadrant.isQuadrantOnLeftSide())
         return true;
      if (icpError.getY() < 0.0 && robotQuadrant.isQuadrantOnRightSide())
         return true;
      if (icpError.getX() > 0.0 && robotQuadrant.isQuadrantInFront())
         return true;
      if (icpError.getX() < 0.0 && robotQuadrant.isQuadrantInHind())
         return true;

      return false;
   }
}
