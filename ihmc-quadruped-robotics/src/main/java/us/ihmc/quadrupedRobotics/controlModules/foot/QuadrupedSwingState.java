package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.trajectory.ThreeDoFSwingFootTrajectory;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedSwingState extends QuadrupedUnconstrainedFootState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean createSwingTrajectoryGraphics = false;

   private final ThreeDoFSwingFootTrajectory swingTrajectory;
   private final FramePoint3D goalPosition = new FramePoint3D();
   private final FramePoint3D initialPosition = new FramePoint3D();
   private final GlitchFilteredYoBoolean touchdownTrigger;

   private final QuadrupedFootControlModuleParameters parameters;

   private final YoBoolean stepCommandIsValid;
   private final YoDouble timestamp;
   private final YoQuadrupedTimedStep currentStepCommand;

   private final ReferenceFrame soleFrame;

   // graphics
   private static final int pointsPerSecondOfSwingTime = 70;
   private static final AppearanceDefinition visualizationAppearance = YoAppearance.Blue();
   private final FramePoint3D swingPositionForVisualization = new FramePoint3D();
   private final BagOfBalls stepSequenceVisualization;

   private boolean triggerSupport;

   public QuadrupedSwingState(RobotQuadrant robotQuadrant, QuadrupedForceControllerToolbox controllerToolbox, QuadrupedSolePositionController solePositionController,
                              YoBoolean stepCommandIsValid, YoQuadrupedTimedStep currentStepCommand, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry registry)
   {
      super(robotQuadrant, controllerToolbox, solePositionController);

      this.stepCommandIsValid = stepCommandIsValid;
      this.timestamp = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();
      this.currentStepCommand = currentStepCommand;

      this.parameters = controllerToolbox.getFootControlModuleParameters();

      soleFrame = controllerToolbox.getReferenceFrames().getSoleFrame(robotQuadrant);

      this.swingTrajectory = new ThreeDoFSwingFootTrajectory(this.robotQuadrant.getPascalCaseName(), registry);
      this.touchdownTrigger = new GlitchFilteredYoBoolean(this.robotQuadrant.getCamelCaseName() + "TouchdownTriggered", registry,
                                                          QuadrupedFootControlModuleParameters.getDefaultTouchdownTriggerWindow());

      // graphics
      if(createSwingTrajectoryGraphics)
      {
         String prefix = robotQuadrant.getPascalCaseName() + "SwingTrajectory";
         stepSequenceVisualization = new BagOfBalls(pointsPerSecondOfSwingTime, 0.005, prefix, visualizationAppearance, registry, graphicsListRegistry);
      }
      else
      {
         stepSequenceVisualization = null;
      }
   }

   @Override
   public void onEntry()
   {
      super.onEntry();

      // initialize swing trajectory
      double groundClearance = currentStepCommand.getGroundClearance();
      TimeInterval timeInterval = currentStepCommand.getTimeInterval();

      initialPosition.setToZero(soleFrame);
      initialPosition.changeFrame(worldFrame);

      currentStepCommand.getGoalPosition(goalPosition);
      goalPosition.changeFrame(worldFrame);
      goalPosition.add(0.0, 0.0, parameters.getStepGoalOffsetZParameter());

      swingTrajectory.initializeTrajectory(initialPosition, goalPosition, groundClearance, timeInterval);

      if(createSwingTrajectoryGraphics)
         updateGraphics(timeInterval.getStartTime(), timeInterval.getEndTime());

      // initialize contact state and feedback gains
      solePositionController.reset();
      solePositionController.getGains().set(parameters.getSolePositionGains());
      solePositionControllerSetpoints.initialize(soleFrame);

      touchdownTrigger.set(false);
      triggerSupport = false;
   }

   private void updateGraphics(double startTime, double endTime)
   {
      stepSequenceVisualization.reset();
      double swingDuration = endTime - startTime;
      int numberOfPoints = MathTools.clamp((int) (pointsPerSecondOfSwingTime * swingDuration), 2, pointsPerSecondOfSwingTime);

      for (int i = 0; i < numberOfPoints; i++)
      {
         double t = startTime + (i + 1) * swingDuration / numberOfPoints;
         swingTrajectory.computeTrajectory(t);
         swingTrajectory.getPosition(swingPositionForVisualization);
         stepSequenceVisualization.setBall(swingPositionForVisualization);
      }
   }

   @Override
   public void doAction(double timeInState)
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

      if(createSwingTrajectoryGraphics)
         updateGraphics(currentTime, currentStepCommand.getTimeInterval().getEndTime());

      soleForceCommand.changeFrame(soleFrame);
      virtualForceCommand.setLinearForce(soleFrame, soleForceCommand);

      super.doControl();

      // Trigger support phase.
      if (currentTime >= touchDownTime)
      {
         if (stepTransitionCallback != null)
         {
            stepTransitionCallback.onTouchDown(robotQuadrant);
         }
         triggerSupport = true;
      }
   }

   @Override
   public QuadrupedFootControlModule.FootEvent fireEvent(double timeInState)
   {
      return triggerSupport ? QuadrupedFootControlModule.FootEvent.TIMEOUT : null;
   }

   @Override
   public void onExit()
   {
      soleForceCommand.setToZero();
      stepCommandIsValid.set(false);

      if(createSwingTrajectoryGraphics)
         stepSequenceVisualization.hideAll();
      triggerSupport = false;
   }
}
