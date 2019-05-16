package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.QuadrupedSupportPolygons;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
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
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.util.ArrayList;
import java.util.List;

public class QuadrupedBalanceBasedStepDelayer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final QuadrantDependentList<? extends PlaneContactState> contactStates;

   private final FrameVector2D icpError = new FrameVector2D();

   private final QuadrupedSupportPolygons supportPolygons;
   private final QuadrupedReferenceFrames referenceFrames;

   private final QuadrantDependentList<YoBoolean> areFeetDelayed = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFramePoint3D> delayedFootLocations = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoDouble> delayDurations = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoDouble> icpErrorInFootDirections = new QuadrantDependentList<>();

   private final List<QuadrupedTimedStep> stepsStarting = new ArrayList<>();

   private final FramePoint3D tempFootPoint = new FramePoint3D();
   private final FrameVector2D vectorToFoot = new FrameVector2D();

   private final BooleanProvider allowDelayingSteps = new BooleanParameter("allowingDelayingSteps", registry, false);
   private final BooleanProvider delayAllSubsequentSteps = new BooleanParameter("delayAllSubsequentSteps", registry, true);
   private final DoubleProvider icpErrorInFootDirectionForDelay = new DoubleParameter("icpErrorInFootDirectionForDelay", registry, 0.01);
   private final DoubleProvider maximumDelayDuration = new DoubleParameter("maximumDelayDuration", registry, 0.1);
   private final IntegerProvider controlTicksToDelay = new IntegerParameter("controlTicksToDelay", registry, 2);

   private final double controlDt;

   public QuadrupedBalanceBasedStepDelayer(QuadrupedControllerToolbox controllerToolbox, YoVariableRegistry parentRegistry)
   {
      supportPolygons = controllerToolbox.getSupportPolygons();
      referenceFrames = controllerToolbox.getReferenceFrames();

      contactStates = controllerToolbox.getFootContactStates();

      controlDt = controllerToolbox.getRuntimeEnvironment().getControlDT();

      YoGraphicsListRegistry graphicsListRegistry = controllerToolbox.getRuntimeEnvironment().getGraphicsListRegistry();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         areFeetDelayed.put(robotQuadrant, new YoBoolean(robotQuadrant.getCamelCaseName() + "_IsFootDelayed", registry));
         delayDurations.put(robotQuadrant, new YoDouble(robotQuadrant.getCamelCaseName() + "_DelayDuration", registry));
         icpErrorInFootDirections.put(robotQuadrant, new YoDouble(robotQuadrant.getCamelCaseName() + "_icpErrorInFootDirection", registry));

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
                                        FrameVector3DReadOnly dcmError, double normalizedDcmEllipticalError)
   {
      if (!allowDelayingSteps.getValue() && normalizedDcmEllipticalError < 1.0)
         return false;

      inactiveSteps.clear();
      for (int i = 0; i < allOtherSteps.size(); i++)
         inactiveSteps.add(allOtherSteps.get(i));


      icpError.setIncludingFrame(dcmError);
      icpError.scale(-1.0);
      icpError.changeFrame(supportPolygons.getMidFeetZUpFrame());

      stepsStarting.clear();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         areFeetDelayed.get(robotQuadrant).set(false);
         if (!contactStates.get(robotQuadrant).inContact())
            delayDurations.get(robotQuadrant).set(0.0);
      }

      for (int i = 0; i < activeSteps.size(); i++)
      {
         QuadrupedTimedStep activeStep = activeSteps.get(i);
         inactiveSteps.remove(activeStep);

         if (contactStates.get(activeStep.getRobotQuadrant()).inContact())
            stepsStarting.add(activeStep);
      }

      double delayAmount = controlTicksToDelay.getValue() * controlDt;
      boolean stepWasDelayed = false;

      for (int i = 0; i < stepsStarting.size(); i++)
      {
         QuadrupedTimedStep stepStarting = stepsStarting.get(i);
         RobotQuadrant quadrantStarting = stepStarting.getRobotQuadrant();

         tempFootPoint.setToZero(referenceFrames.getSoleFrame(quadrantStarting));
         tempFootPoint.changeFrame(supportPolygons.getMidFeetZUpFrame());
         vectorToFoot.setIncludingFrame(tempFootPoint);

         YoDouble icpErrorInFootDirection = icpErrorInFootDirections.get(quadrantStarting);
         icpErrorInFootDirection.set(icpError.dot(vectorToFoot));

         if (icpErrorInFootDirection.getDoubleValue() > icpErrorInFootDirectionForDelay.getValue())
         {
            YoDouble delayDuration = delayDurations.get(quadrantStarting);

            if (delayDuration.getDoubleValue() + delayAmount < maximumDelayDuration.getValue())
            {
               TimeIntervalBasics timeInterval = stepStarting.getTimeInterval();
               double currentStartTime = timeInterval.getStartTime();
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
         {
            QuadrupedTimedStep inactiveStep = inactiveSteps.get(i);
            double startTime = inactiveStep.getTimeInterval().getStartTime();
            inactiveStep.getTimeInterval().setStartTime(startTime + delayAmount);
         }
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FixedFramePoint3DBasics delayedFootLocation = delayedFootLocations.get(robotQuadrant);
         if (areFeetDelayed.get(robotQuadrant).getBooleanValue())
         {
            tempFootPoint.setToZero(referenceFrames.getSoleFrame(robotQuadrant));
            tempFootPoint.changeFrame(worldFrame);
            delayedFootLocation.set(tempFootPoint);
         }
         else
         {
            delayedFootLocation.setToNaN();
         }
      }

      return stepWasDelayed;
   }
}
