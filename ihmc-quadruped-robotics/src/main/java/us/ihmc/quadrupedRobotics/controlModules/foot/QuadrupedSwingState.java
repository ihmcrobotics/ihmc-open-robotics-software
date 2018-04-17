package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualForceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPositionTrajectoryGenerator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.trajectory.OneWaypointSwingGenerator;
import us.ihmc.robotics.dataStructures.parameters.FrameParameterVector3D;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.math.trajectories.*;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.robotics.trajectories.providers.CurrentRigidBodyStateProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class QuadrupedSwingState extends QuadrupedFootState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final OneWaypointSwingGenerator swingTrajectoryWaypointCalculator;
   private final MultipleWaypointsBlendedPositionTrajectoryGenerator blendedSwingTrajectory;
   private final SoftTouchdownPositionTrajectoryGenerator touchdownTrajectory;

   private final FrameEuclideanTrajectoryPoint tempPositionTrajectoryPoint = new FrameEuclideanTrajectoryPoint();

   private final CurrentRigidBodyStateProvider currentStateProvider;

   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FrameVector3D initialLinearVelocity = new FrameVector3D();

   private final FramePoint3D finalPosition = new FramePoint3D();
   private final FrameVector3D finalLinearVelocity = new FrameVector3D();

   private final GlitchFilteredYoBoolean touchdownTrigger;

   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameVector3D desiredVelocity = new FrameVector3D();

   private final YoFramePoint3D desiredSolePosition;
   private final YoFrameVector3D desiredSoleLinearVelocity;

   private final QuadrupedFootControlModuleParameters parameters;

   private final YoDouble swingDuration;
   private final YoDouble timeInState;
   private final YoBoolean stepCommandIsValid;
   private final YoDouble timestamp;
   private final YoQuadrupedTimedStep currentStepCommand;

   private final VirtualForceCommand virtualForceCommand = new VirtualForceCommand();
   private final PointFeedbackControlCommand feedbackControlCommand = new PointFeedbackControlCommand();

   private final MovingReferenceFrame soleFrame;

   private final QuadrupedControllerToolbox controllerToolbox;
   private final RobotQuadrant robotQuadrant;

   private boolean triggerSupport;

   private final FrameVector3DReadOnly touchdownVelocity;
   private final FrameVector3DReadOnly touchdownAcceleration;

   public QuadrupedSwingState(RobotQuadrant robotQuadrant, QuadrupedControllerToolbox controllerToolbox, YoBoolean stepCommandIsValid,
                              YoQuadrupedTimedStep currentStepCommand, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry registry)
   {
      this.robotQuadrant = robotQuadrant;
      this.controllerToolbox = controllerToolbox;

      this.stepCommandIsValid = stepCommandIsValid;
      this.timestamp = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();
      this.currentStepCommand = currentStepCommand;

      String namePrefix = robotQuadrant.getPascalCaseName();

      timeInState = new YoDouble(namePrefix + "TimeInState", registry);
      swingDuration = new YoDouble(namePrefix + "SwingDuration", registry);

      Vector3D defaultTouchdownVelocity = new Vector3D(0.0, 0.0, 0.0);
      touchdownVelocity = new FrameParameterVector3D(namePrefix + "TouchdownVelocity", ReferenceFrame.getWorldFrame(), defaultTouchdownVelocity, registry);
      touchdownAcceleration = new FrameParameterVector3D(namePrefix + "TouchdownAcceleration", ReferenceFrame.getWorldFrame(), defaultTouchdownVelocity, registry);

      this.parameters = controllerToolbox.getFootControlModuleParameters();

      soleFrame = controllerToolbox.getReferenceFrames().getSoleFrame(robotQuadrant);

      swingTrajectoryWaypointCalculator = new OneWaypointSwingGenerator(this.robotQuadrant.getPascalCaseName(), 0.5, 0.04, 0.3, registry, graphicsListRegistry);

      MultipleWaypointsPositionTrajectoryGenerator baseTrajectory = new MultipleWaypointsPositionTrajectoryGenerator(this.robotQuadrant.getPascalCaseName(),
                                                                                                                     true, worldFrame, registry);
      blendedSwingTrajectory = new MultipleWaypointsBlendedPositionTrajectoryGenerator(this.robotQuadrant.getPascalCaseName(), baseTrajectory, worldFrame,
                                                                                       registry);
      touchdownTrajectory = new SoftTouchdownPositionTrajectoryGenerator(namePrefix, registry);

      currentStateProvider = new CurrentRigidBodyStateProvider(soleFrame);

      this.touchdownTrigger = new GlitchFilteredYoBoolean(this.robotQuadrant.getCamelCaseName() + "TouchdownTriggered", registry,
                                                          QuadrupedFootControlModuleParameters.getDefaultTouchdownTriggerWindow());

      RigidBody foot = controllerToolbox.getFullRobotModel().getFoot(robotQuadrant);
      FramePoint3D currentPosition = new FramePoint3D(soleFrame);
      currentPosition.changeFrame(foot.getBodyFixedFrame());

      feedbackControlCommand.set(controllerToolbox.getFullRobotModel().getBody(), foot);
      feedbackControlCommand.setBodyFixedPointToControl(currentPosition);

      desiredSolePosition = new YoFramePoint3D(namePrefix + "DesiredSolePositionInWorld", worldFrame, registry);
      desiredSoleLinearVelocity = new YoFrameVector3D(namePrefix + "DesiredSoleLinearVelocityInWorld", worldFrame, registry);
   }

   @Override
   public void onEntry()
   {
      timeInState.set(0.0);
      controllerToolbox.getFootContactState(robotQuadrant).clear();

      // initialize swing trajectory
      currentStateProvider.getPosition(initialPosition);
      currentStateProvider.getLinearVelocity(initialLinearVelocity);
      initialPosition.changeFrame(worldFrame);
      initialLinearVelocity.changeFrame(worldFrame);

      currentStepCommand.getGoalPosition(finalPosition);

      finalPosition.changeFrame(worldFrame);
      finalPosition.addZ(parameters.getStepGoalOffsetZParameter());

      swingDuration.set(currentStepCommand.getTimeInterval().getDuration());

      fillAndInitializeTrajectories();

      touchdownTrigger.set(false);
      triggerSupport = false;
   }

   @Override
   public void doAction(double timeInState)
   {
      double currentTime = timestamp.getDoubleValue();
      double touchDownTime = currentStepCommand.getTimeInterval().getEndTime();

      this.timeInState.set(timeInState);

      blendForStepAdjustment(timeInState);

      PositionTrajectoryGenerator activeTrajectory;
      if (timeInState > swingDuration.getDoubleValue())
         activeTrajectory = touchdownTrajectory;
      else
         activeTrajectory = blendedSwingTrajectory;

      activeTrajectory.compute(timeInState);
      activeTrajectory.getPosition(desiredPosition);
      activeTrajectory.getVelocity(desiredVelocity);

      desiredSolePosition.setMatchingFrame(desiredPosition);
      desiredSoleLinearVelocity.setMatchingFrame(desiredVelocity);

      // Detect early touch-down. // FIXME do something else with this (trigger a loaded transition?)
      FrameVector3D soleForceEstimate = controllerToolbox.getTaskSpaceEstimates().getSoleVirtualForce(robotQuadrant);
      soleForceEstimate.changeFrame(worldFrame);
      double pressureEstimate = -soleForceEstimate.getZ();
      double normalizedTimeInSwing = timeInState / currentStepCommand.getTimeInterval().getDuration();
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
         soleForceCommand.changeFrame(soleFrame);
         virtualForceCommand.setLinearForce(soleFrame, soleForceCommand);

      }
      else
      {
         feedbackControlCommand.set(desiredPosition, desiredVelocity);
         feedbackControlCommand.setGains(parameters.getSolePositionGains());
      }

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

   private void fillAndInitializeTrajectories()
   {
      blendedSwingTrajectory.clearTrajectory(worldFrame);
      blendedSwingTrajectory.appendPositionWaypoint(0.0, initialPosition, initialLinearVelocity);

      finalLinearVelocity.setIncludingFrame(touchdownVelocity);

      swingTrajectoryWaypointCalculator.setInitialConditions(initialPosition, initialLinearVelocity);
      swingTrajectoryWaypointCalculator.setFinalConditions(finalPosition, finalLinearVelocity);
      swingTrajectoryWaypointCalculator.setStepTime(swingDuration.getDoubleValue());
      swingTrajectoryWaypointCalculator.setTrajectoryType(TrajectoryType.DEFAULT);
      swingTrajectoryWaypointCalculator.setSwingHeight(currentStepCommand.getGroundClearance());
      swingTrajectoryWaypointCalculator.initialize();

      for (int i = 0; i < swingTrajectoryWaypointCalculator.getNumberOfWaypoints(); i++)
      {
         swingTrajectoryWaypointCalculator.getWaypointData(i, tempPositionTrajectoryPoint);
         blendedSwingTrajectory.appendPositionWaypoint(tempPositionTrajectoryPoint);
      }

      blendedSwingTrajectory.appendPositionWaypoint(swingDuration.getDoubleValue(), finalPosition, finalLinearVelocity);

      blendedSwingTrajectory.initializeTrajectory();
      blendedSwingTrajectory.initialize();

      touchdownTrajectory.setLinearTrajectory(swingDuration.getDoubleValue(), finalPosition, finalLinearVelocity, touchdownAcceleration);
      touchdownTrajectory.initialize();
   }

   private void blendForStepAdjustment(double timeInState)
   {
      double duration = swingDuration.getDoubleValue();

      // Compute current goal position.
      currentStepCommand.getGoalPosition(finalPosition);
      finalPosition.changeFrame(worldFrame);
      finalPosition.addZ(parameters.getStepGoalOffsetZParameter());

      // Compute swing trajectory.
      if (duration - timeInState > parameters.getMinimumStepAdjustmentTimeParameter())
      {
         blendedSwingTrajectory.clear();

         touchdownTrajectory.setLinearTrajectory(duration, finalPosition, finalLinearVelocity, touchdownAcceleration);
         touchdownTrajectory.initialize();

         blendedSwingTrajectory.blendFinalConstraint(finalPosition, duration, duration);
         blendedSwingTrajectory.initialize();
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
      stepCommandIsValid.set(false);
      swingDuration.setToNaN();
      timeInState.setToNaN();

      triggerSupport = false;

      desiredSolePosition.setToNaN();
      desiredSoleLinearVelocity.setToNaN();
      swingTrajectoryWaypointCalculator.hideVisualization();
   }

   @Override
   public VirtualModelControlCommand<?> getVirtualModelControlCommand()
   {
      if (touchdownTrigger.getBooleanValue())
         return virtualForceCommand;
      else
         return null;
   }

   @Override
   public PointFeedbackControlCommand getFeedbackControlCommand()
   {
      if (touchdownTrigger.getBooleanValue())
         return null;
      else
         return feedbackControlCommand;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return feedbackControlCommand;
   }
}
