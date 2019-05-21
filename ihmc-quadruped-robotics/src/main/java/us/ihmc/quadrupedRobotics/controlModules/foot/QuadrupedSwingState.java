package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.trajectories.SwingGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointSwingGenerator;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.commonWalkingControlModules.trajectories.OneWaypointSwingGenerator;
import us.ihmc.quadrupedRobotics.util.YoQuadrupedTimedStep;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.math.trajectories.MultipleWaypointsBlendedPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.robotics.trajectories.providers.CurrentRigidBodyStateProvider;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

public class QuadrupedSwingState extends QuadrupedFootState
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final FrameVector3DReadOnly zeroVector3D = new FrameVector3D(worldFrame);
   private static final boolean debug = false;

   private final OneWaypointSwingGenerator oneWaypointSwingTrajectoryCalculator;
   private final TwoWaypointSwingGenerator twoWaypointSwingTrajectoryCalculator;
   private final MultipleWaypointsBlendedPositionTrajectoryGenerator blendedSwingTrajectory;
   private final SoftTouchdownPositionTrajectoryGenerator touchdownTrajectory;

   private final FrameEuclideanTrajectoryPoint tempPositionTrajectoryPoint = new FrameEuclideanTrajectoryPoint();

   private final CurrentRigidBodyStateProvider currentStateProvider;

   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FrameVector3D initialLinearVelocity = new FrameVector3D();

   private final YoFramePoint3D finalPosition;
   private final FrameVector3D finalLinearVelocity = new FrameVector3D();

   private final FramePoint3D lastStepPosition = new FramePoint3D();

   private final GlitchFilteredYoBoolean touchdownTrigger;

   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameVector3D desiredVelocity = new FrameVector3D();
   private final FrameVector3D desiredAcceleration = new FrameVector3D();

   private final YoEnum<TrajectoryType> activeTrajectoryType;

   private final YoFramePoint3D desiredSolePosition;
   private final YoFrameVector3D desiredSoleLinearVelocity;
   private final YoFrameVector3D desiredSoleLinearAcceleration;

   private final QuadrupedFootControlModuleParameters parameters;

   private final double controlDT;

   private final YoBoolean stepCommandIsValid;

   private final YoDouble swingDuration;
   private final YoDouble timeInState;
   private final YoDouble timeInStateWithSwingSpeedUp;
   private final YoDouble timeRemainingInState;
   private final YoDouble timeRemainingInStateWithSwingSpeedUp;
   private final DoubleProvider timestamp;
   private final YoQuadrupedTimedStep currentStepCommand;
   private final YoBoolean hasMinimumTimePassed;

   private final YoBoolean isSwingPastDone;

   private WholeBodyControllerCoreMode controllerCoreMode = WholeBodyControllerCoreMode.VIRTUAL_MODEL;
   private final PointFeedbackControlCommand feedbackControlCommand = new PointFeedbackControlCommand();

   private final QuadrupedControllerToolbox controllerToolbox;
   private final RobotQuadrant robotQuadrant;

   private final FrameVector3DReadOnly touchdownVelocity;
   private final FrameVector3DReadOnly touchdownAcceleration;

   private final FootSwitchInterface footSwitch;

   private final YoDouble swingTimeSpeedUpFactor;
   private final YoDouble maxSwingTimeSpeedUpFactor;
   private final BooleanProvider isSwingSpeedUpEnabled;

   public QuadrupedSwingState(RobotQuadrant robotQuadrant, QuadrupedControllerToolbox controllerToolbox, YoBoolean stepCommandIsValid,
                              YoQuadrupedTimedStep currentStepCommand, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry registry)
   {
      this.robotQuadrant = robotQuadrant;
      this.controllerToolbox = controllerToolbox;

      this.stepCommandIsValid = stepCommandIsValid;
      this.timestamp = controllerToolbox.getRuntimeEnvironment().getRobotTimestamp();
      this.currentStepCommand = currentStepCommand;
      this.controlDT = controllerToolbox.getRuntimeEnvironment().getControlDT();

      footSwitch = controllerToolbox.getRuntimeEnvironment().getFootSwitches().get(robotQuadrant);

      String namePrefix = robotQuadrant.getPascalCaseName();

      timeInState = new YoDouble(namePrefix + "TimeInState", registry);
      timeInStateWithSwingSpeedUp = new YoDouble(namePrefix + "TimeInStateWithSwingSpeedUp", registry);
      timeRemainingInState = new YoDouble(namePrefix + "TimeRemainingInState", registry);
      timeRemainingInStateWithSwingSpeedUp = new YoDouble(namePrefix + "TimeRemainingInStateWithSwingSpeedUp", registry);
      swingDuration = new YoDouble(namePrefix + "SwingDuration", registry);
      hasMinimumTimePassed = new YoBoolean(namePrefix + "HasMinimumTimePassed", registry);

      swingTimeSpeedUpFactor = new YoDouble(namePrefix + "TimeSpeedUpFactor", registry);
      maxSwingTimeSpeedUpFactor = new YoDouble(namePrefix + "MaxTimeSpeedUpFactor", registry);

      isSwingPastDone = new YoBoolean(namePrefix + "IsSwingPastDone", registry);

      this.parameters = controllerToolbox.getFootControlModuleParameters();

      touchdownVelocity = parameters.getTouchdownVelocity();
      touchdownAcceleration = parameters.getTouchdownAcceleration();

      isSwingSpeedUpEnabled = parameters.getIsSwingSpeedUpEnabled();

      finalPosition = new YoFramePoint3D(namePrefix + "StepFinalPosition", worldFrame, registry);

      finalPosition.setToNaN();
      lastStepPosition.setToNaN();
      activeTrajectoryType = new YoEnum<>(namePrefix + TrajectoryType.class.getSimpleName(), registry, TrajectoryType.class);

      MovingReferenceFrame soleFrame = controllerToolbox.getReferenceFrames().getSoleFrame(robotQuadrant);

      //      swingTrajectoryWaypointCalculator = new OneWaypointSwingGenerator(namePrefix, 0.5, 0.04, 0.3, registry, graphicsListRegistry);

      double minSwingHeight = 0.04;
      double maxSwingHeight = 0.3;
      double defaultSwingHeight = 0.04;

      oneWaypointSwingTrajectoryCalculator = new OneWaypointSwingGenerator(namePrefix + "1", minSwingHeight, maxSwingHeight, defaultSwingHeight, registry,
                                                                           graphicsListRegistry);
      twoWaypointSwingTrajectoryCalculator = new TwoWaypointSwingGenerator(namePrefix + "2", minSwingHeight, maxSwingHeight, defaultSwingHeight, registry,
                                                                           graphicsListRegistry);
      FramePoint3D dummyPoint = new FramePoint3D();
      dummyPoint.setToNaN();
      twoWaypointSwingTrajectoryCalculator.setStanceFootPosition(dummyPoint);

      MultipleWaypointsPositionTrajectoryGenerator baseTrajectory = new MultipleWaypointsPositionTrajectoryGenerator(this.robotQuadrant.getPascalCaseName(),
                                                                                                                     worldFrame, registry);
      blendedSwingTrajectory = new MultipleWaypointsBlendedPositionTrajectoryGenerator(this.robotQuadrant.getPascalCaseName(), baseTrajectory, worldFrame,
                                                                                       registry);
      touchdownTrajectory = new SoftTouchdownPositionTrajectoryGenerator(namePrefix, registry);

      currentStateProvider = new CurrentRigidBodyStateProvider(soleFrame);

      this.touchdownTrigger = new GlitchFilteredYoBoolean(this.robotQuadrant.getCamelCaseName() + "TouchdownTriggered", registry,
                                                          QuadrupedFootControlModuleParameters.getDefaultTouchdownTriggerWindow());

      RigidBodyBasics foot = controllerToolbox.getFullRobotModel().getFoot(robotQuadrant);
      FramePoint3D currentPosition = new FramePoint3D(soleFrame);
      currentPosition.changeFrame(foot.getBodyFixedFrame());

      feedbackControlCommand.set(controllerToolbox.getFullRobotModel().getBody(), foot);
      feedbackControlCommand.setBodyFixedPointToControl(currentPosition);

      desiredSolePosition = new YoFramePoint3D(namePrefix + "DesiredSolePositionInWorld", worldFrame, registry);
      desiredSoleLinearVelocity = new YoFrameVector3D(namePrefix + "DesiredSoleLinearVelocityInWorld", worldFrame, registry);
      desiredSoleLinearAcceleration = new YoFrameVector3D(namePrefix + "DesiredSoleLinearAccelerationInWorld", worldFrame, registry);

      YoGraphicPosition finalGraphic = new YoGraphicPosition(namePrefix + "FinalPosition", finalPosition, 0.02, YoAppearance.Red());
      YoGraphicPosition desiredGraphic = new YoGraphicPosition(namePrefix + "DesiredPosition", desiredSolePosition, 0.015, YoAppearance.Green());
      graphicsListRegistry.registerYoGraphic("SwingState", finalGraphic);
      graphicsListRegistry.registerYoGraphic("SwingState", desiredGraphic);
      graphicsListRegistry.registerArtifact("SwingState", finalGraphic.createArtifact());
   }

   public void setControllerCoreMode(WholeBodyControllerCoreMode controllerCoreMode)
   {
      this.controllerCoreMode = controllerCoreMode;
      feedbackControlCommand.setControlMode(controllerCoreMode);
   }

   @Override
   public void onEntry()
   {
      timeInState.set(0.0);
      swingTimeSpeedUpFactor.set(1.0);
      timeInStateWithSwingSpeedUp.setToNaN();
      timeRemainingInState.setToNaN();
      timeRemainingInStateWithSwingSpeedUp.setToNaN();

      controllerToolbox.getFootContactState(robotQuadrant).clear();
      footSwitch.reset();

      lastStepPosition.setMatchingFrame(finalPosition);
      if (lastStepPosition.containsNaN())
         lastStepPosition.setToZero(controllerToolbox.getSoleReferenceFrame(robotQuadrant));

      finalPosition.set(currentStepCommand.getReferenceFrame(), currentStepCommand.getGoalPosition());

      if (stepTransitionCallback != null)
      {
         stepTransitionCallback.onLiftOff(currentStepCommand);
      }

      // initialize swing trajectory
      currentStateProvider.getPosition(initialPosition);
      currentStateProvider.getLinearVelocity(initialLinearVelocity);
      initialPosition.changeFrame(worldFrame);
      initialLinearVelocity.changeFrame(worldFrame);

      finalPosition.addZ(parameters.getStepGoalOffsetZParameter());

      double swingDuration = Math
            .max(currentStepCommand.getTimeInterval().getEndTime() - timestamp.getValue(), parameters.getMinSwingTimeForDisturbanceRecovery());
      setFootstepDurationInternal(swingDuration);

      activeTrajectoryType.set(TrajectoryType.DEFAULT);

      if (checkStepUpOrDown(finalPosition))
         activeTrajectoryType.set(TrajectoryType.OBSTACLE_CLEARANCE);

      fillAndInitializeTrajectories(true);

      touchdownTrigger.set(false);
      isSwingPastDone.set(false);

      if (debug)
      {
         PrintTools.debug(
               currentStepCommand.getRobotQuadrant() + ", " + new Point3D(currentStepCommand.getGoalPosition()) + ", " + currentStepCommand.getGroundClearance()
                     + ", " + currentStepCommand.getTimeInterval());
      }
   }

   private boolean checkStepUpOrDown(FramePoint3DReadOnly finalPosition)
   {
      lastStepPosition.changeFrame(worldFrame);
      lastStepPosition.checkReferenceFrameMatch(finalPosition);
      double zDifference = Math.abs(finalPosition.getZ() - lastStepPosition.getZ());
      return zDifference > parameters.getMinHeightDifferenceForObstacleClearance();
   }

   private void setFootstepDurationInternal(double swingTime)
   {
      swingDuration.set(swingTime);
      maxSwingTimeSpeedUpFactor.set(Math.max(swingTime / parameters.getMinSwingTimeForDisturbanceRecovery(), 1.0));
   }

   @Override
   public void doAction(double timeInState)
   {
      this.timeInState.set(timeInState);
      timeRemainingInState.set(swingDuration.getDoubleValue() - timeInState);

      blendForStepAdjustment();

      double time;
      if (!isSwingSpeedUpEnabled.getValue() || timeInStateWithSwingSpeedUp.isNaN())
      {
         time = timeInState;
      }
      else
      {
         timeInStateWithSwingSpeedUp.add(swingTimeSpeedUpFactor.getDoubleValue() * controlDT);
         time = timeInStateWithSwingSpeedUp.getDoubleValue();
         timeRemainingInStateWithSwingSpeedUp.set(swingDuration.getDoubleValue() - time);
      }

      PositionTrajectoryGenerator activeTrajectory;
      if (!timeRemainingInStateWithSwingSpeedUp.isNaN() && timeRemainingInStateWithSwingSpeedUp.getValue() < 0.0)
         activeTrajectory = touchdownTrajectory;
      else if (timeRemainingInState.getValue() < 0.0)
         activeTrajectory = touchdownTrajectory;
      else
         activeTrajectory = blendedSwingTrajectory;

      SwingGenerator waypointGenerator =
            activeTrajectoryType.getEnumValue() == TrajectoryType.DEFAULT ? oneWaypointSwingTrajectoryCalculator : twoWaypointSwingTrajectoryCalculator;
      if (activeTrajectoryType.getEnumValue() != TrajectoryType.DEFAULT && waypointGenerator.doOptimizationUpdate()) // haven't finished original planning
         fillAndInitializeTrajectories(false);

      activeTrajectory.compute(time);
      activeTrajectory.getPosition(desiredPosition);
      activeTrajectory.getVelocity(desiredVelocity);
      activeTrajectory.getAcceleration(desiredAcceleration);

      if (isSwingSpeedUpEnabled.getValue() && !timeInStateWithSwingSpeedUp.isNaN())
      {
         desiredVelocity.scale(swingTimeSpeedUpFactor.getDoubleValue());

         double speedUpFactorSquared = swingTimeSpeedUpFactor.getDoubleValue() * swingTimeSpeedUpFactor.getDoubleValue();
         desiredAcceleration.scale(speedUpFactorSquared);
      }

      desiredSolePosition.setMatchingFrame(desiredPosition);
      desiredSoleLinearVelocity.setMatchingFrame(desiredVelocity);
      desiredSoleLinearAcceleration.setMatchingFrame(desiredAcceleration);

      if (controllerCoreMode == WholeBodyControllerCoreMode.INVERSE_DYNAMICS)
         feedbackControlCommand.setInverseDynamics(desiredPosition, desiredVelocity, desiredAcceleration);
      else if (controllerCoreMode == WholeBodyControllerCoreMode.VIRTUAL_MODEL)
         feedbackControlCommand.setVirtualModelControl(desiredPosition, desiredVelocity, zeroVector3D);
      else
         throw new UnsupportedOperationException("Unsupported control mode: " + controllerCoreMode);
      feedbackControlCommand.setGains(parameters.getSolePositionGains());
      feedbackControlCommand.setWeightsForSolver(parameters.getSolePositionWeights());

      updateEndOfStateConditions(time);
   }

   private boolean hasMinimumTimePassed(double timeInState)
   {
      return timeInState / swingDuration.getDoubleValue() > parameters.getMinPhaseThroughSwingForContact();
   }

   private void updateEndOfStateConditions(double timeInState)
   {
      // Detect early touch-down.
      hasMinimumTimePassed.set(hasMinimumTimePassed(timeInState));
      if (hasMinimumTimePassed.getBooleanValue())
      {
         touchdownTrigger.update(footSwitch.hasFootHitGround());
      }

      double currentTime = timestamp.getValue();
      double touchDownTime = currentStepCommand.getTimeInterval().getEndTime();
      double startTime = currentStepCommand.getTimeInterval().getStartTime();
      double percentDone = (currentTime - startTime) / (touchDownTime - startTime);

      // Trigger support phase.
      if (percentDone >= (1.0 + parameters.getPercentPastSwingForDone()))
      {
         isSwingPastDone.set(true);
      }
   }

   private void fillAndInitializeTrajectories(boolean initializeOptimizer)
   {
      blendedSwingTrajectory.clearTrajectory(worldFrame);
      blendedSwingTrajectory.appendPositionWaypoint(0.0, initialPosition, initialLinearVelocity);

      finalLinearVelocity.setIncludingFrame(touchdownVelocity);

      SwingGenerator waypointCalculator;
      if (activeTrajectoryType.getEnumValue() == TrajectoryType.DEFAULT)
         waypointCalculator = oneWaypointSwingTrajectoryCalculator;
      else
         waypointCalculator = twoWaypointSwingTrajectoryCalculator;

      if (initializeOptimizer)
      {
         waypointCalculator.setInitialConditions(initialPosition, initialLinearVelocity);
         waypointCalculator.setFinalConditions(finalPosition, finalLinearVelocity);
         waypointCalculator.setStepTime(swingDuration.getDoubleValue());
         waypointCalculator.setTrajectoryType(activeTrajectoryType.getEnumValue());
         waypointCalculator.setSwingHeight(currentStepCommand.getGroundClearance());
         if (activeTrajectoryType.getEnumValue() == TrajectoryType.DEFAULT)
            oneWaypointSwingTrajectoryCalculator.setWaypointProportion(parameters.getFlatSwingWaypointProportion());

         else if (activeTrajectoryType.getEnumValue() == TrajectoryType.OBSTACLE_CLEARANCE)
            twoWaypointSwingTrajectoryCalculator
                  .setWaypointProportions(parameters.getSwingObstacleClearanceWaypointProportion0(), parameters.getSwingObstacleClearanceWaypointProportion1());
         else
            twoWaypointSwingTrajectoryCalculator.setWaypointProportions(parameters.getSwingWaypointProportion0(), parameters.getSwingWaypointProportion1());
      }

      for (int i = 0; i < waypointCalculator.getNumberOfWaypoints(); i++)
      {
         waypointCalculator.getWaypointData(i, tempPositionTrajectoryPoint);
         blendedSwingTrajectory.appendPositionWaypoint(tempPositionTrajectoryPoint);
      }

      blendedSwingTrajectory.appendPositionWaypoint(swingDuration.getDoubleValue(), finalPosition, finalLinearVelocity);

      blendedSwingTrajectory.initializeTrajectory();
      blendedSwingTrajectory.initialize();

      touchdownTrajectory.setLinearTrajectory(swingDuration.getDoubleValue(), finalPosition, finalLinearVelocity, touchdownAcceleration);
      touchdownTrajectory.initialize();
   }

   private void blendForStepAdjustment()
   {
      // Compute current goal position.
      finalPosition.set(currentStepCommand.getReferenceFrame(), currentStepCommand.getGoalPosition());
      finalPosition.addZ(parameters.getStepGoalOffsetZParameter());

      double duration = swingDuration.getDoubleValue();
      blendedSwingTrajectory.clear();

      touchdownTrajectory.setLinearTrajectory(duration, finalPosition, finalLinearVelocity, touchdownAcceleration);
      touchdownTrajectory.initialize();

      blendedSwingTrajectory.blendFinalConstraint(finalPosition, duration, parameters.getFractionOfSwingForBlending() * duration);
      blendedSwingTrajectory.initialize();
   }

   /**
    * Request the swing trajectory to speed up using the given speed up factor.
    * It is clamped w.r.t. to {@param #getMinimumSwingTimeForDisturbanceRecovery}.
    * @param speedUpTime
    * @return the current swing time remaining for the swing foot trajectory
    */
   public double requestSwingSpeedUp(double speedUpTime)
   {
      if (timeRemainingInState.isNaN())
         return Double.NaN;

      double remainingTime = timeRemainingInState.getDoubleValue() - speedUpTime;

      double speedUpFactor;
      if (remainingTime > 1.0e-3)
      {
         speedUpFactor = timeRemainingInState.getDoubleValue() / remainingTime;
      }
      else
      {
         speedUpFactor = Double.POSITIVE_INFINITY;
      }

      if (isSwingSpeedUpEnabled.getValue() && (speedUpFactor > parameters.getMinRequiredSpeedUpFactor() && speedUpFactor > swingTimeSpeedUpFactor
            .getDoubleValue()))
      {
         speedUpFactor = MathTools.clamp(speedUpFactor, swingTimeSpeedUpFactor.getDoubleValue(), maxSwingTimeSpeedUpFactor.getDoubleValue());

         swingTimeSpeedUpFactor.set(speedUpFactor);
         if (timeInStateWithSwingSpeedUp.isNaN())
            timeInStateWithSwingSpeedUp.set(timeInState.getDoubleValue());
      }

      return computeSwingTimeRemaining(timeInState.getDoubleValue());
   }

   private double computeSwingTimeRemaining(double timeInState)
   {
      double swingDuration = this.swingDuration.getDoubleValue();
      if (!timeInStateWithSwingSpeedUp.isNaN())
      {
         double swingTimeRemaining = (swingDuration - timeInStateWithSwingSpeedUp.getDoubleValue()) / swingTimeSpeedUpFactor.getDoubleValue();
         return swingTimeRemaining;
      }
      else
      {
         return swingDuration - timeInState;
      }
   }

   public double computeClampedSpeedUpTime(double speedUpTimeRequested)
   {
      double remainingTime;
      double timeRemainingInState;

      if (timeRemainingInStateWithSwingSpeedUp.isNaN())
         timeRemainingInState = this.timeRemainingInState.getDoubleValue();
      else
         timeRemainingInState = timeRemainingInStateWithSwingSpeedUp.getDoubleValue();

      remainingTime = timeRemainingInState - speedUpTimeRequested;

      double speedUpFactor;
      if (remainingTime > 1.0e-3)
      {
         speedUpFactor = this.timeRemainingInState.getDoubleValue() / remainingTime;
         if (isSwingSpeedUpEnabled.getValue() && (speedUpFactor > 1.1 && speedUpFactor > swingTimeSpeedUpFactor.getDoubleValue()))
         {
            speedUpFactor = MathTools.clamp(speedUpFactor, swingTimeSpeedUpFactor.getDoubleValue(), maxSwingTimeSpeedUpFactor.getDoubleValue());
         }
         else
         {
            return 0.0;
         }
      }
      else
      {
         return 0.0;
      }

      remainingTime = this.timeRemainingInState.getDoubleValue() / speedUpFactor;

      double clampedSpeedUpTime = timeRemainingInState - remainingTime;
      return clampedSpeedUpTime;
   }

   @Override
   public QuadrupedFootControlModule.FootEvent fireEvent(double timeInState)
   {
      QuadrupedFootControlModule.FootEvent eventToReturn = null;
      if (isSwingPastDone.getBooleanValue())
         eventToReturn = QuadrupedFootControlModule.FootEvent.TIMEOUT;
      if (touchdownTrigger.getBooleanValue())
         eventToReturn = QuadrupedFootControlModule.FootEvent.LOADED;

      if (eventToReturn != null && stepTransitionCallback != null)
         stepTransitionCallback.onTouchDown(robotQuadrant);

      return eventToReturn;
   }

   @Override
   public void onExit()
   {
      stepCommandIsValid.set(false);
      swingDuration.setToNaN();
      timeInState.setToNaN();
      swingTimeSpeedUpFactor.setToNaN();
      timeRemainingInState.setToNaN();
      timeInStateWithSwingSpeedUp.setToNaN();

      desiredSolePosition.setToNaN();
      desiredSoleLinearVelocity.setToNaN();
      oneWaypointSwingTrajectoryCalculator.hideVisualization();
      twoWaypointSwingTrajectoryCalculator.hideVisualization();
   }

   @Override
   public PointFeedbackControlCommand getFeedbackControlCommand()
   {
      return feedbackControlCommand;
   }

   @Override
   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return feedbackControlCommand;
   }
}
