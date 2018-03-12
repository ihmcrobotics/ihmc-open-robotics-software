package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionControllerSetpoints;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.trajectory.ThreeDoFSwingFootTrajectory;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedSwingState extends QuadrupedFootState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final RobotQuadrant robotQuadrant;
   private final ThreeDoFSwingFootTrajectory swingTrajectory;
   private final FramePoint3D goalPosition = new FramePoint3D();
   private final FramePoint3D initialPosition = new FramePoint3D();
   private final GlitchFilteredYoBoolean touchdownTrigger;

   private final QuadrupedFootControlModuleParameters parameters;

   private final YoBoolean stepCommandIsValid;
   private final YoDouble timestamp;
   private final YoQuadrupedTimedStep currentStepCommand;

   private final QuadrupedSolePositionController solePositionController;
   private final QuadrupedSolePositionControllerSetpoints solePositionControllerSetpoints;

   private final QuadrupedForceControllerToolbox controllerToolbox;

   private final ReferenceFrame soleFrame;

   // graphics
   private static final int numberOfPointsToVisualize = 16;
   private static final AppearanceDefinition visualizationAppearance = YoAppearance.Blue();
   private final FramePoint3D swingPositionForVisualization = new FramePoint3D();
   private final BagOfBalls stepSequenceVisualization;

   public QuadrupedSwingState(RobotQuadrant robotQuadrant, QuadrupedForceControllerToolbox controllerToolbox, QuadrupedSolePositionController solePositionController,
                              YoBoolean stepCommandIsValid, YoQuadrupedTimedStep currentStepCommand, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry registry)
   {
      this.robotQuadrant = robotQuadrant;
      this.controllerToolbox = controllerToolbox;
      this.solePositionController = solePositionController;
      this.stepCommandIsValid = stepCommandIsValid;
      this.timestamp = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();
      this.currentStepCommand = currentStepCommand;

      this.parameters = controllerToolbox.getFootControlModuleParameters();
      this.solePositionControllerSetpoints = new QuadrupedSolePositionControllerSetpoints(robotQuadrant);

      soleFrame = controllerToolbox.getReferenceFrames().getSoleFrame(robotQuadrant);

      this.swingTrajectory = new ThreeDoFSwingFootTrajectory(this.robotQuadrant.getPascalCaseName(), registry);
      this.touchdownTrigger = new GlitchFilteredYoBoolean(this.robotQuadrant.getCamelCaseName() + "TouchdownTriggered", registry,
                                                          QuadrupedFootControlModuleParameters.getDefaultTouchdownTriggerWindow());

      // graphics
      String prefix = robotQuadrant.getPascalCaseName() + "SwingTrajectory";
      stepSequenceVisualization = new BagOfBalls(numberOfPointsToVisualize, 0.015, prefix, visualizationAppearance, registry, graphicsListRegistry);
   }

   @Override
   public void onEntry()
   {
      // initialize swing trajectory
      double groundClearance = currentStepCommand.getGroundClearance();
      TimeInterval timeInterval = currentStepCommand.getTimeInterval();

      initialPosition.setToZero(soleFrame);
      initialPosition.changeFrame(worldFrame);

      currentStepCommand.getGoalPosition(goalPosition);
      goalPosition.changeFrame(worldFrame);
      goalPosition.add(0.0, 0.0, parameters.getStepGoalOffsetZParameter());

      goalPosition.setX(initialPosition.getX());
      goalPosition.setY(initialPosition.getY());

      swingTrajectory.initializeTrajectory(initialPosition, goalPosition, groundClearance, timeInterval);
      updateGraphics(timeInterval);

      // initialize contact state and feedback gains
      solePositionController.reset();
      solePositionController.getGains().set(parameters.getSolePositionGains());
      solePositionControllerSetpoints.initialize(soleFrame);

      touchdownTrigger.set(false);
   }

   private void updateGraphics(TimeInterval timeInterval)
   {
      stepSequenceVisualization.reset();
      double swingDuration = timeInterval.getDuration();

      for (int i = 0; i < numberOfPointsToVisualize; i++)
      {
         double t = timeInterval.getStartTime() + (i + 1) * swingDuration / numberOfPointsToVisualize;
         swingTrajectory.computeTrajectory(t);
         swingTrajectory.getPosition(swingPositionForVisualization);
         stepSequenceVisualization.setBall(swingPositionForVisualization);
      }
   }

   @Override
   public QuadrupedFootControlModule.FootEvent process()
   {
      double currentTime = timestamp.getDoubleValue();
      double touchDownTime = currentStepCommand.getTimeInterval().getEndTime();

      // Compute current goal position.
      currentStepCommand.getGoalPosition(goalPosition);
      goalPosition.changeFrame(worldFrame);
      goalPosition.add(0.0, 0.0, parameters.getStepGoalOffsetZParameter());

      // Compute swing trajectory.
      if (touchDownTime - currentTime > parameters.getMinimumStepAdjustmentTimeParameter())
      {
         swingTrajectory.adjustTrajectory(goalPosition, currentTime);
      }
      swingTrajectory.computeTrajectory(currentTime);
      swingTrajectory.getPosition(solePositionControllerSetpoints.getSolePosition());

      // Detect early touch-down. // FIXME do something else with this (trigger a loaded transition?)
      FrameVector3D soleForceEstimate = controllerToolbox.getTaskSpaceEstimates().getSoleVirtualForce(robotQuadrant);
      soleForceEstimate.changeFrame(worldFrame);
      double pressureEstimate = -soleForceEstimate.getZ();
      double relativeTimeInSwing = currentTime - currentStepCommand.getTimeInterval().getStartTime();
      double normalizedTimeInSwing = relativeTimeInSwing / currentStepCommand.getTimeInterval().getDuration();
      if (normalizedTimeInSwing > 0.5)
      {
         touchdownTrigger.update(pressureEstimate > parameters.getTouchdownPressureLimitParameter());
      }

      // Compute sole force.
      if (touchdownTrigger.getBooleanValue())
      {
         double pressureLimit = parameters.getTouchdownPressureLimitParameter();
         soleForceCommand.changeFrame(worldFrame);
         soleForceCommand.set(0, 0, -pressureLimit);
      }
      else
      {
         solePositionController.compute(soleForceCommand, solePositionControllerSetpoints, controllerToolbox.getTaskSpaceEstimates().getSoleLinearVelocity(robotQuadrant));
         soleForceCommand.changeFrame(worldFrame);
      }

      // Trigger support phase.
      if (currentTime >= touchDownTime)
      {
         if (stepTransitionCallback != null)
         {
            stepTransitionCallback.onTouchDown(robotQuadrant);
         }
         return QuadrupedFootControlModule.FootEvent.TIMEOUT;
      }
      else
         return null;
   }

   @Override
   public void onExit()
   {
      soleForceCommand.setToZero();
      stepCommandIsValid.set(false);
      stepSequenceVisualization.hideAll();
   }
}
