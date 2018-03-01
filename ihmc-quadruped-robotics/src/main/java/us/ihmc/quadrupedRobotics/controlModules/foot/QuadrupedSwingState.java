package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionControllerSetpoints;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.trajectory.ThreeDoFSwingFootTrajectory;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
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

   public QuadrupedSwingState(RobotQuadrant robotQuadrant, QuadrupedForceControllerToolbox controllerToolbox, QuadrupedSolePositionController solePositionController,
                              YoBoolean stepCommandIsValid, YoQuadrupedTimedStep currentStepCommand, YoVariableRegistry registry)
   {
      this.robotQuadrant = robotQuadrant;
      this.controllerToolbox = controllerToolbox;
      this.solePositionController = solePositionController;
      this.stepCommandIsValid = stepCommandIsValid;
      this.timestamp = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();
      this.currentStepCommand = currentStepCommand;

      this.parameters = controllerToolbox.getFootControlModuleParameters();
      this.solePositionControllerSetpoints = new QuadrupedSolePositionControllerSetpoints(robotQuadrant);

      soleFrame = controllerToolbox.getReferenceFrames().getFootFrame(robotQuadrant);

      this.swingTrajectory = new ThreeDoFSwingFootTrajectory(this.robotQuadrant.getPascalCaseName(), registry);
      this.touchdownTrigger = new GlitchFilteredYoBoolean(this.robotQuadrant.getCamelCaseName() + "TouchdownTriggered", registry,
                                                          parameters.getTouchdownTriggerWindowParameter());
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

      swingTrajectory.initializeTrajectory(initialPosition, goalPosition, groundClearance, timeInterval);

      // initialize contact state and feedback gains
      solePositionController.reset();
      solePositionController.getGains().setProportionalGains(parameters.getSolePositionProportionalGainsParameter());
      solePositionController.getGains().setDerivativeGains(parameters.getSolePositionDerivativeGainsParameter());
      solePositionController.getGains()
                            .setIntegralGains(parameters.getSolePositionIntegralGainsParameter(), parameters.getSolePositionMaxIntegralErrorParameter());
      solePositionControllerSetpoints.initialize(soleFrame);

      touchdownTrigger.set(false);
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
   }
}
