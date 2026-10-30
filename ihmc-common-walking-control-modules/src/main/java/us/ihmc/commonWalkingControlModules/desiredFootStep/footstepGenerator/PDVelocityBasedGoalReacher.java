package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicArrow3D;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicPoint3D;

import controller_msgs.ControllerWalkToGoalStatusMessage;
import controller_msgs.ControllerWaypointStatusMessage;
import controller_msgs.VelocityBasedWalkingInputMessage;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ControllerReleaseGoalCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ControllerWaypointGoalCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ControllerWaypointGoalListCommand;
import us.ihmc.commons.DeadbandTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

/**
 * Class uses a clamped PD controller that aligns headings to reach a goal.
 */
public class PDVelocityBasedGoalReacher implements Updatable, SCS2YoGraphicHolder
{
   private static final double BASE_POSITION_Z_OFFSET_FOR_VISUALIZATION = 0.75;
   private static final double GOAL_POSITION_Z_OFFSET_FOR_VISUALIZATION = 0.1;

   // TODO extract these in to a parameter file.
   private static final double DEFAULT_MAX_RADIAL_ACCELERATION = 1.0;
   private static final double DEFAULT_MAX_HEADING_RATE = Math.PI / 2.0;

   private static final double DEFAULT_MAX_FORWARD_SPEED = 0.5;
   private static final double DEFAULT_MAX_BACKWARD_SPEED = 0.3;
   private static final double DEFAULT_MAX_LATERAL_SPEED = 0.4;
   private static final double DEFAULT_MAX_TURNING_SPEED = 0.5;
   private static final double DEFAULT_MIN_SPEED = 0.1;
   private static final double DEFAULT_MIN_TURNING_SPEED = 0.15;
   /**
    * Below this translation scale a yaw command is treated as a pivot. The walking policy falls on
    * {@code walk=true} with little forward speed, so those commands become a walking arc.
    */
   private static final double DEFAULT_MIN_TRANSLATION_SCALE_WHILE_TURNING = 0.70;
   /** Body-frame forward speed used for a walking turn. */
   private static final double DEFAULT_MIN_FORWARD_SPEED_WHILE_TURNING = 0.25;
   /**
    * Tightest curve commanded while turning. At {@link #DEFAULT_MIN_FORWARD_SPEED_WHILE_TURNING} this
    * caps yaw at about 0.21 rad/s, what the walking policy can track without locking the knees.
    */
   private static final double DEFAULT_MIN_TURN_RADIUS_WHILE_WALKING = 1.2;

   private static final double DEFAULT_K_ANGLE = 1.5;
   private static final double DEFAULT_ANGLE_DEADBAND = Math.toRadians(3.0);

   private static final double DEFAULT_DISTANCE_TO_GOAL_THRESHOLD_TO_STOP = 0.1;
   private static final double DEFAULT_ANGLE_TO_GOAL_THRESHOLD_TO_STOP = Math.toRadians(5.0);

   private static final double DEFAULT_RAMP_DOWN_SPEED_THRESHOLD = 0.05;
   private static final double DEFAULT_RAMP_DOWN_DECAY_RATE = 0.95;

   private static final double DEFAULT_DISTANCE_TO_MATCH_GOAL_ANGLE = 0.75;
   private static final double DEFAULT_DISTANCE_TO_FACE_GOAL = 1.5;

   /** A waypoint whose walking heading is further off than this starts with a turn in place. */
   private static final double DEFAULT_HEADING_ERROR_TO_START_TURN_IN_PLACE = Math.toRadians(60.0);
   /** A turn in place to the walking heading ends inside this error. */
   private static final double DEFAULT_HEADING_ERROR_TO_END_TURN_IN_PLACE = Math.toRadians(5.0);
   /** Yaw allowed while walking with too little forward speed for a walking arc, e.g. sideways or backward. */
   private static final double DEFAULT_MAX_TURNING_SPEED_WHILE_WALKING_SLOWLY = 0.15;
   /** Turn-walk-turn steers to a point this far ahead along the start-to-goal line. */
   private static final double DEFAULT_PATH_LOOKAHEAD_DISTANCE = 0.5;
   /**
    * While turning to the goal yaw, the robot walks back to the goal only past this many times the position proximity.
    * The walking policy carries on 0.2 to 0.3 m when the walk stops and the turn starts; the position hold brings it back.
    */
   private static final double DEFAULT_POSITION_PROXIMITY_SCALE_WHILE_TURNING_TO_GOAL_YAW = 3.5;
   /** Gain (1/s) and speed cap (m/s) for stepping back to the goal while turning to the goal yaw. */
   private static final double DEFAULT_POSITION_HOLD_GAIN = 1.0;
   private static final double DEFAULT_MAX_POSITION_HOLD_SPEED = 0.1;

   /**
    * Each waypoint is reached in up to three phases. Large heading changes turn in place, at no more than
    * {@code maxTurningSpeed}, instead of walking an arc: an arc that has to turn the robot around walks it in loops,
    * and at the goal there is no forward speed left to turn with.
    */
   private enum Phase
   {
      /** Turning in place to the heading the robot will walk with. */
      TURN_TO_PATH,
      /** Walking to the goal position. */
      WALK,
      /** At the goal position, turning in place to the goal yaw. */
      TURN_TO_GOAL_YAW
   }

   private final RecyclingArrayList<GoalWaypoint> goalPoses = new RecyclingArrayList<>(GoalWaypoint::new);
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final GoalReacherWaypointVisualizer waypointVisualizer;

   private final YoFramePose3D currentGoalPose = new YoFramePose3D("goalPose", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D currentGoalDirection = new YoFrameVector3D("goalDirection", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose3D currentPose = new YoFramePose3D("currentPose", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D currentDirection = new YoFrameVector3D("currentDirection", ReferenceFrame.getWorldFrame(), registry);

   /** Pose of the robot at the moment it began walking toward the current waypoint. */
   private final YoFramePose3D startPose = new YoFramePose3D("startPose", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D startDirection = new YoFrameVector3D("startDirection", ReferenceFrame.getWorldFrame(), registry);
   /** Vector from startPose to the current goal, used to draw the path line. */
   private final YoFrameVector3D startToGoalVector = new YoFrameVector3D("startToGoalVector", ReferenceFrame.getWorldFrame(), registry);

   // Frame of the root link to track to the goal
   private final ReferenceFrame frameOfTheBase;

   // control parameters
   private double lastTickTime = Double.NaN;
   private final YoDouble maxRadialAcceleration = new YoDouble("maxRadialAcceleration", registry);
   private final YoDouble maxHeadingRate = new YoDouble("maxHeadingRate", registry);
   private double computedDt;

   private final YoDouble desiredCruisingSpeedScalar = new YoDouble("desiredCruisingSpeedScalar", registry);
   /**
    * Minimum normalized speed the robot may travel through the current intermediate waypoint.
    * Derived from the dot product of the incoming and outgoing leg directions: 1.0 means the robot
    * can pass through at full speed (legs are collinear); 0.0 means the robot must stop (turn >= 90deg
    * or this is the terminal waypoint).
    */
   private final YoDouble waypointThroughputSpeedScalar = new YoDouble("waypointThroughputSpeedScalar", registry);

   private final YoDouble maxForwardSpeed = new YoDouble("maxForwardSpeed", registry);
   private final YoDouble maxLateralSpeed = new YoDouble("maxLateralSpeed", registry);
   private final YoDouble maxBackwardSpeed = new YoDouble("maxBackwardSpeed", registry);
   private final YoDouble maxTurningSpeed = new YoDouble("maxTurningSpeed", registry);
   private final YoDouble minSpeed = new YoDouble("minSpeed", registry);
   private final YoDouble minTurningSpeed = new YoDouble("minTurningSpeed", registry);
   private final YoDouble minTranslationScaleWhileTurning = new YoDouble("minTranslationScaleWhileTurning", registry);
   private final YoDouble minForwardSpeedWhileTurning = new YoDouble("minForwardSpeedWhileTurning", registry);
   private final YoDouble minTurnRadiusWhileWalking = new YoDouble("minTurnRadiusWhileWalking", registry);

   private final YoDouble kAngle = new YoDouble("kAngle", registry);
   private final YoDouble angleDeadband = new YoDouble("angleDeadband", registry);
   private final YoDouble distanceToMatchGoalAngle = new YoDouble("distanceToMatchGoalAngle", registry);
   private final YoDouble distanceToFaceGoal = new YoDouble("distanceToFaceGoal", registry);

   // Termination Conditions
   private final YoDouble distanceToGoalThresholdToStop = new YoDouble("distanceToGoalThresholdToStop", registry);
   private final YoDouble angleToGoalThresholdToStop = new YoDouble("angleToGoalThresholdToStop", registry);

   // Measures of error
   private final YoDouble distanceToGoal = new YoDouble("distanceToGoal", registry);
   private final YoDouble angleToGoal = new YoDouble("angleToGoal", registry);

   private final YoVector2D vectorToGoalInPelvisFrame = new YoVector2D("vectorToGoalInPelvisFrame", registry);
   private final YoDouble angleToHeading = new YoDouble("angleToHeading", registry);
   private final YoDouble rateLimitedAngleToHeading = new YoDouble("rateLimitedAngleToHeading", registry);

   private final YoBoolean hasGoal = new YoBoolean("hasGoal", registry);
   private final YoBoolean shouldHoldGoal = new YoBoolean("shouldHoldGoal", registry);
   private final YoBoolean hasReachedGoal = new YoBoolean("hasReachedGoal", registry);
   private final YoBoolean wasGoalReached = new YoBoolean("wasGoalReached", registry);
   private final YoBoolean goalOrientationMatters = new YoBoolean("goalOrientationMatters", registry);
   /**
    * When true, do not face the goal past {@link #distanceToFaceGoal}. Spoken reverse and sidestep
    * use this so a 2 m backward walk stays a reverse instead of a U-turn.
    */
   private final YoBoolean keepHeading = new YoBoolean("keepHeading", registry);
   /** When true, turn in place to face along the start-to-goal line, walk it, then turn in place to the goal yaw. */
   private final YoBoolean turnWalkTurn = new YoBoolean("turnWalkTurn", registry);
   private final YoEnum<Phase> phase = new YoEnum<>("goalReacherPhase", registry, Phase.class);
   /** Set once a turn in place to the walking heading has started, so it runs to the smaller end threshold. */
   private final YoBoolean isTurningToPath = new YoBoolean("isTurningToPath", registry);
   private final YoDouble walkingHeadingYaw = new YoDouble("walkingHeadingYaw", registry);
   private final YoDouble headingErrorToStartTurnInPlace = new YoDouble("headingErrorToStartTurnInPlace", registry);
   private final YoDouble headingErrorToEndTurnInPlace = new YoDouble("headingErrorToEndTurnInPlace", registry);
   private final YoDouble maxTurningSpeedWhileWalkingSlowly = new YoDouble("maxTurningSpeedWhileWalkingSlowly", registry);
   private final YoDouble pathLookaheadDistance = new YoDouble("pathLookaheadDistance", registry);
   private final YoDouble positionProximityScaleWhileTurningToGoalYaw = new YoDouble("positionProximityScaleWhileTurningToGoalYaw", registry);
   private final YoDouble positionHoldGain = new YoDouble("positionHoldGain", registry);
   private final YoDouble maxPositionHoldSpeed = new YoDouble("maxPositionHoldSpeed", registry);
   private final YoBoolean isRampingDownAfterGoal = new YoBoolean("isRampingDownAfterGoal", registry);
   private final YoDouble rampDownDecayRate = new YoDouble("rampDownDecayRate", registry);
   private final YoDouble rampDownDecay = new YoDouble("rampDownDecay", registry);
   private final YoDouble rampDownSpeedThreshold = new YoDouble("rampDownSpeedThreshold", registry);
   private final YoBoolean pendingWaypointReachedPublication = new YoBoolean("pendingWaypointReachedPublication", registry);
   private final FramePose2D pendingWaypointReachedPose = new FramePose2D();

   // Feedback terms
   private final YoDouble normalizedRadialSpeed;
   private final YoDouble rateLimitedNormalizedRadialSpeed;
   // CLF terms
   private final YoFramePoint3D basePosition = new YoFramePoint3D("basePosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoVector2D desiredVelocity = new YoVector2D("desiredVelocity", registry);
   private final YoDouble desiredSpeed = new YoDouble("desiredSpeed", registry);
   private final YoFrameVector3D desiredLinearVelocity = new YoFrameVector3D("desiredLinearVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D desiredAngularVelocity = new YoFrameVector3D("desiredAngularVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final VelocityBasedWalkingInputMessage outputMessage = new VelocityBasedWalkingInputMessage();

   // Temp Variables
   private final Vector2D vectorToGoal = new Vector2D();

   private final StatusMessageOutputManager statusMessageOutputManager;

   public PDVelocityBasedGoalReacher(ReferenceFrame frameOfBase,
                                     StatusMessageOutputManager statusMessageOutputManager,
                                     YoRegistry parentRegistry)
   {
      this.frameOfTheBase = frameOfBase;
      this.statusMessageOutputManager = statusMessageOutputManager;

      normalizedRadialSpeed = new YoDouble("normalizedRadialSpeed", registry);
      rateLimitedNormalizedRadialSpeed = new YoDouble("rateLimitedNormalizedRadialSpeed", registry);

      maxRadialAcceleration.set(DEFAULT_MAX_RADIAL_ACCELERATION);
      maxHeadingRate.set(DEFAULT_MAX_HEADING_RATE);

      maxForwardSpeed.set(DEFAULT_MAX_FORWARD_SPEED);
      maxBackwardSpeed.set(DEFAULT_MAX_BACKWARD_SPEED);
      maxLateralSpeed.set(DEFAULT_MAX_LATERAL_SPEED);
      maxTurningSpeed.set(DEFAULT_MAX_TURNING_SPEED);
      minSpeed.set(DEFAULT_MIN_SPEED);
      minTurningSpeed.set(DEFAULT_MIN_TURNING_SPEED);
      minTranslationScaleWhileTurning.set(DEFAULT_MIN_TRANSLATION_SCALE_WHILE_TURNING);
      minForwardSpeedWhileTurning.set(DEFAULT_MIN_FORWARD_SPEED_WHILE_TURNING);
      minTurnRadiusWhileWalking.set(DEFAULT_MIN_TURN_RADIUS_WHILE_WALKING);

      kAngle.set(DEFAULT_K_ANGLE);
      angleDeadband.set(DEFAULT_ANGLE_DEADBAND);

      distanceToGoalThresholdToStop.set(DEFAULT_DISTANCE_TO_GOAL_THRESHOLD_TO_STOP);
      angleToGoalThresholdToStop.set(DEFAULT_ANGLE_TO_GOAL_THRESHOLD_TO_STOP);

      distanceToMatchGoalAngle.set(DEFAULT_DISTANCE_TO_MATCH_GOAL_ANGLE);
      distanceToFaceGoal.set(DEFAULT_DISTANCE_TO_FACE_GOAL);
      rampDownSpeedThreshold.set(DEFAULT_RAMP_DOWN_SPEED_THRESHOLD);

      rampDownDecayRate.set(DEFAULT_RAMP_DOWN_DECAY_RATE);

      headingErrorToStartTurnInPlace.set(DEFAULT_HEADING_ERROR_TO_START_TURN_IN_PLACE);
      headingErrorToEndTurnInPlace.set(DEFAULT_HEADING_ERROR_TO_END_TURN_IN_PLACE);
      maxTurningSpeedWhileWalkingSlowly.set(DEFAULT_MAX_TURNING_SPEED_WHILE_WALKING_SLOWLY);
      pathLookaheadDistance.set(DEFAULT_PATH_LOOKAHEAD_DISTANCE);
      positionProximityScaleWhileTurningToGoalYaw.set(DEFAULT_POSITION_PROXIMITY_SCALE_WHILE_TURNING_TO_GOAL_YAW);
      positionHoldGain.set(DEFAULT_POSITION_HOLD_GAIN);
      maxPositionHoldSpeed.set(DEFAULT_MAX_POSITION_HOLD_SPEED);

      parentRegistry.addChild(registry);
      waypointVisualizer = new GoalReacherWaypointVisualizer(registry);
   }

   private final FramePose2D previousGoalPose = new FramePose2D();

   public void consumeNewWaypointList(ControllerWaypointGoalListCommand command)
   {
      goalPoses.clear();
      for (int i = 0; i < command.getNumberOfWaypoints(); i++)
      {
         consumeNewWaypoint(command.getWaypoint(i));
      }
   }

   public void consumeNewWaypoint(ControllerWaypointGoalCommand command)
   {
      if (hasReachedGoal.getBooleanValue() && goalPoses.size() == 1)
      {  // we were holding the previous pose, based on the command. remove it, and add the next one.
         goalPoses.clear();
      }
      // Update the previous goal pose, if we have one. If not, grab the current root pose.
      if (!goalPoses.isEmpty())
      {
         previousGoalPose.set(goalPoses.getLast().getGoalPose());
      }
      else
      {
         previousGoalPose.setFromReferenceFrame(frameOfTheBase);
      }
      // Add the new goal pose.
      goalPoses.add().set(command, previousGoalPose, maxForwardSpeed.getValue());
      if (goalPoses.size() == 1)
      {
         publishWaypointStatus(goalPoses.getFirst().getGoalPose(), 0, true);
         wasGoalReached.set(false);
         captureStartPose();
      }
      hasGoal.set(true);
      hasReachedGoal.set(false); // We want to set this to false to force it to evaluate the new goal points
      shouldHoldGoal.set(command.getShouldHoldPosition());
      isRampingDownAfterGoal.set(false);
      pendingWaypointReachedPublication.set(false);

      distanceToGoalThresholdToStop.set(command.getPositionProximity());
      angleToGoalThresholdToStop.set(command.getOrientationProximity());
   }

   public void consumeReleaseGoalCommand(ControllerReleaseGoalCommand command)
   {
      if (command.getReleaseGoal())
      {
         goalPoses.clear();
         shouldHoldGoal.set(false);
         hasGoal.set(false);
         isRampingDownAfterGoal.set(false);
         pendingWaypointReachedPublication.set(false);
      }
   }

   public void clear()
   {
      hasReachedGoal.set(true);
      hasGoal.set(false);
      keepHeading.set(false);
      turnWalkTurn.set(false);
      phase.set(Phase.TURN_TO_PATH);
      isTurningToPath.set(false);
      goalPoses.clear();
      isRampingDownAfterGoal.set(false);
      pendingWaypointReachedPublication.set(false);
      rateLimitedNormalizedRadialSpeed.set(0.0);
      rateLimitedAngleToHeading.set(0.0);
   }


   private final ControllerWalkToGoalStatusMessage statusMessage = new ControllerWalkToGoalStatusMessage();
   private final ControllerWaypointStatusMessage waypointReachedMessage = new ControllerWaypointStatusMessage();

   @Override
   public void update(double time)
   {
      currentPose.setFromReferenceFrame(frameOfTheBase);

      basePosition.set(currentPose.getPosition());
      basePosition.addZ(BASE_POSITION_Z_OFFSET_FOR_VISUALIZATION);

      // Snapshot state before updateGoalPose() so we can detect transitions for status messages.
      boolean hasGoal = this.hasGoal.getBooleanValue();
      boolean hasReachedGoal = this.hasReachedGoal.getBooleanValue();
      // The ramp-down ends one tick after hasReachedGoal is set, and only that tick reports the goal as reached.
      boolean wasRampingDownAfterGoal = isRampingDownAfterGoal.getBooleanValue();
      if (Double.isNaN(lastTickTime))
         computedDt = 0.0;
      else
         computedDt = time - lastTickTime;
      lastTickTime = time;

      updateGoalPose();
      waypointVisualizer.update(goalPoses.size(), i -> goalPoses.get(i).getGoalPose());

      if (this.hasGoal.getBooleanValue() && !this.hasReachedGoal.getBooleanValue())
      {
         // Update this value for visualization.
         currentGoalDirection.set(1.0, 0.0, 0.0);
         currentGoalPose.getOrientation().transform(currentGoalDirection);

         currentPose.getPosition().setZ(GOAL_POSITION_Z_OFFSET_FOR_VISUALIZATION);
         currentDirection.set(1.0, 0.0, 0.0);
         currentPose.getOrientation().transform(currentDirection);

         startDirection.set(1.0, 0.0, 0.0);
         startPose.getOrientation().transform(startDirection);
         startToGoalVector.set(currentGoalPose.getX() - startPose.getX(),
                               currentGoalPose.getY() - startPose.getY(),
                               currentGoalPose.getZ() - startPose.getZ());

         computeVelocities(currentGoalPose);
         updateOutputMessage();
      }

      if (isRampingDownAfterGoal.getBooleanValue())
      {
         rampDownDecay.set(rampDownDecay.getDoubleValue() * rampDownDecayRate.getDoubleValue());
         // Decrease speed at the same rate as the acceleration limit to avoid a step change to zero.
         normalizedRadialSpeed.set(0.0);

         desiredVelocity.scale(rampDownDecay.getDoubleValue());
         desiredAngularVelocity.scale(rampDownDecay.getDoubleValue());

         if (desiredVelocity.norm() <= rampDownSpeedThreshold.getValue() && desiredAngularVelocity.norm() <= rampDownSpeedThreshold.getValue())
         {
            // Don't set this for one more tick. That way, we get this last output message ,and we return the "stop" message once.
            if (this.hasReachedGoal.getValue())
               isRampingDownAfterGoal.set(false);
            this.hasReachedGoal.set(true);
            if (pendingWaypointReachedPublication.getBooleanValue())
            {
               publishWaypointStatus(pendingWaypointReachedPose, 0, false);
               pendingWaypointReachedPublication.set(false);
            }
            rateLimitedAngleToHeading.set(0.0);
            rateLimitedNormalizedRadialSpeed.set(0.0);
            desiredVelocity.setToZero();
            desiredAngularVelocity.setToZero();
         }
         updateOutputMessage();
      }
      else if (!this.hasGoal.getBooleanValue())
      {
         startDirection.setToNaN();
         startToGoalVector.setToNaN();
      }
      // else: holding at goal after ramp-down - nothing to do.

      if (!isRampingDownAfterGoal.getBooleanValue())
         rampDownDecay.set(1.0);

      if (hasGoal || this.hasGoal.getBooleanValue() || (hasReachedGoal != this.hasReachedGoal.getValue()) || wasRampingDownAfterGoal)
      {
         // Send status.
         statusMessage.getCurrentPosition().set(currentPose.getPosition());
         statusMessage.setCurrentGoalXPosition(currentGoalPose.getX());
         statusMessage.setCurrentGoalYPosition(currentGoalPose.getY());
         statusMessage.setCurrentGoalYaw(currentGoalPose.getYaw());
         statusMessage.setIsReached(this.hasReachedGoal.getBooleanValue() && !isRampingDownAfterGoal.getBooleanValue());
         reportWalkToGoalStatus();
      }
   }

   private void reportWalkToGoalStatus()
   {
      statusMessageOutputManager.reportStatusMessage(statusMessage);
   }

   public VelocityBasedWalkingInputMessage getOutputMessage()
   {
      // Deliver the command while actively pursuing a goal or ramping down after reaching one.
      if (hasGoal.getBooleanValue() && (!hasReachedGoal.getBooleanValue() || wasGoalReached.getBooleanValue()))
         return outputMessage;
      if (isRampingDownAfterGoal.getBooleanValue())
         return outputMessage;
      return null;
   }

   private final Vector2D outgoingDirection = new Vector2D();

   private void updateGoalPose()
   {
      // If we have no goal poses in queue, don't do anything.
      if (goalPoses.isEmpty())
      {
         hasGoal.set(false);
         currentGoalPose.setToNaN();
         // Don't declare "reached" while still decelerating; the ramp-down block will set it.
         if (!isRampingDownAfterGoal.getBooleanValue())
         {
            hasReachedGoal.set(true);
            rateLimitedNormalizedRadialSpeed.set(0.0);
            rateLimitedAngleToHeading.set(0.0);
         }
         return;
      }

      // Advance through any waypoints that have already been reached, stopping at the first unmet one.
      // After the loop, reachedGoal reflects whether the current front waypoint is met.
      boolean reachedGoal = true;
      while (!goalPoses.isEmpty() && reachedGoal)
      {
         reachedGoal = reachedGoal(goalPoses.getFirst().getGoalPose(), goalPoses.getFirst().getGoalOrientationMatters());
         if (reachedGoal)
         {
            if (goalPoses.size() > 1)
            {
               // Completed an intermediate waypoint; publish status and advance to the next one.
               publishWaypointStatus(goalPoses.getFirst().getGoalPose(), goalPoses.size() - 1, false);
               goalPoses.remove(0);
               publishWaypointStatus(goalPoses.getFirst().getGoalPose(), goalPoses.size() - 1, true);
               wasGoalReached.set(false); // reset so the next waypoint's reached-status can be published
               captureStartPose();
            }
            else if (goalPoses.size() == 1 && !shouldHoldGoal.getBooleanValue())
            {
               // Completed the final waypoint and we're not holding; begin ramp-down.
               // hasReachedGoal will be set true only after the robot has fully stopped.
               pendingWaypointReachedPose.set(goalPoses.getFirst().getGoalPose());
               pendingWaypointReachedPublication.set(true);
               goalPoses.remove(0);
               wasGoalReached.set(false);
               isRampingDownAfterGoal.set(true);
            }
            else if (shouldHoldGoal.getBooleanValue())
            {
               // Holding at the final waypoint; keep it in the list so the robot stays.
               // Trigger ramp-down only once (first time reaching), not on every subsequent tick.
               if (!wasGoalReached.getBooleanValue())
                  isRampingDownAfterGoal.set(true);
               break;
            }
         }
      }

      hasGoal.set(!goalPoses.isEmpty());

      // Defer "waypoint reached" until ramp-down finishes so the status message is not
      // published while the robot is still decelerating toward a stop.
      // wasGoalReached persists across ticks so we only arm the deferred publication once.
      if (reachedGoal && !wasGoalReached.getBooleanValue() && !goalPoses.isEmpty())
      {
         pendingWaypointReachedPose.set(goalPoses.getFirst().getGoalPose());
         pendingWaypointReachedPublication.set(true);
      }

      // Don't set hasReachedGoal=true while ramping down to a stop; the ramp-down block sets it.
      if (!isRampingDownAfterGoal.getBooleanValue())
         hasReachedGoal.set(reachedGoal);
      wasGoalReached.set(reachedGoal);

      if (hasGoal.getBooleanValue())
      {
         GoalWaypoint waypoint = goalPoses.getFirst();

         currentGoalPose.checkReferenceFrameMatch(waypoint.getGoalPose().getReferenceFrame());
         currentGoalPose.getPosition().set(waypoint.getGoalPose().getPosition(), GOAL_POSITION_Z_OFFSET_FOR_VISUALIZATION);
         currentGoalPose.getOrientation().setToYawOrientation(waypoint.getGoalPose().getYaw());

         distanceToGoalThresholdToStop.set(waypoint.getProximityToReachGoal());
         angleToGoalThresholdToStop.set(waypoint.getAngleToReachGoal());
         desiredCruisingSpeedScalar.set(waypoint.getCruisingSpeedScaler());
         goalOrientationMatters.set(waypoint.getGoalOrientationMatters());
         keepHeading.set(waypoint.getKeepHeading());
         turnWalkTurn.set(waypoint.getTurnWalkTurn());

         // Compute how fast the robot may pass through this waypoint without stopping.
         // Dot product of the incoming leg direction with the outgoing leg direction gives
         // cos(turn angle): 1 = straight, 0 = 90 degree turn, -1 = U-turn.
         // Clamp to [0, 1]: turns >= 90 degrees require a full stop; smaller turns allow proportionally
         // higher throughput speed. Terminal waypoints always use 0 (robot must stop).
         if (goalPoses.size() >= 2)
         {
            GoalWaypoint nextWaypoint = goalPoses.get(1);
            outgoingDirection.sub(nextWaypoint.getGoalPose().getPosition(), waypoint.getGoalPose().getPosition());
            double length = outgoingDirection.norm();
            if (length > 1e-6)
            {
               outgoingDirection.scale(1.0 / length);
               double dot = waypoint.getIncomingDirection().dot(outgoingDirection);
               waypointThroughputSpeedScalar.set(Math.max(0.0, dot));
            }
            else
            {
               waypointThroughputSpeedScalar.set(0.0);
            }
         }
         else
         {
            waypointThroughputSpeedScalar.set(0.0);
         }
      }
      else
      {
         currentGoalPose.setToNaN();
         keepHeading.set(false);
         turnWalkTurn.set(false);
      }
   }

   private void captureStartPose()
   {
      startPose.setFromReferenceFrame(frameOfTheBase);
      startPose.getPosition().setZ(GOAL_POSITION_Z_OFFSET_FOR_VISUALIZATION);
      phase.set(Phase.TURN_TO_PATH);
      isTurningToPath.set(false);
   }

   private void publishWaypointStatus(FramePose2DReadOnly pose, int waypointsRemaining, boolean isStarted)
   {
      waypointReachedMessage.setGoalXPosition(pose.getX());
      waypointReachedMessage.setGoalYPosition(pose.getY());
      waypointReachedMessage.setGoalYaw(pose.getYaw());
      waypointReachedMessage.setWaypointsRemaining(waypointsRemaining);
      waypointReachedMessage.setIsStarted(isStarted);
      statusMessageOutputManager.reportStatusMessage(waypointReachedMessage);
   }

   private boolean reachedGoal(FramePose2DReadOnly goalPose, boolean orientationMatters)
   {
      distanceToGoal.set(currentPose.getPosition().distanceXY(goalPose.getPosition()));
      angleToGoal.set(AngleTools.computeAngleDifferenceMinusPiToPi(currentPose.getYaw(), goalPose.getYaw()));

      boolean positionReached = distanceToGoal.getDoubleValue() < distanceToGoalThresholdToStop.getValue();
      boolean orientationReached = !orientationMatters || Math.abs(angleToGoal.getValue()) < angleToGoalThresholdToStop.getValue();
      return positionReached && orientationReached;
   }

   /**
    * Computes the walking command for the current phase. A waypoint starts in {@link Phase#TURN_TO_PATH}, which turns
    * in place only when the walking heading is far off (always, for turn-walk-turn), then walks, then turns in place to
    * the goal yaw once at the goal position if the goal yaw matters.
    */
   private void computeVelocities(FramePose3DReadOnly goalPose)
   {
      vectorToGoal.set(goalPose.getX() - currentPose.getX(), goalPose.getY() - currentPose.getY());
      currentPose.getOrientation().inverseTransform(vectorToGoal, vectorToGoalInPelvisFrame);
      double distance = vectorToGoal.norm();
      double positionProximity = distanceToGoalThresholdToStop.getValue();

      if (phase.getValue() == Phase.TURN_TO_GOAL_YAW && distance > positionProximityScaleWhileTurningToGoalYaw.getValue() * positionProximity)
         phase.set(Phase.WALK); // Pushed off the goal while turning
      if (phase.getValue() == Phase.WALK && goalOrientationMatters.getValue() && distance < positionProximity)
         phase.set(Phase.TURN_TO_GOAL_YAW);

      if (phase.getValue() == Phase.TURN_TO_PATH)
      {
         walkingHeadingYaw.set(computeWalkingHeadingYaw(goalPose, distance));
         double headingError = AngleTools.computeAngleDifferenceMinusPiToPi(walkingHeadingYaw.getValue(), currentPose.getYaw());
         double errorToTurn;
         if (isTurningToPath.getValue() || turnWalkTurn.getValue())
            errorToTurn = headingErrorToEndTurnInPlace.getValue();
         else
            errorToTurn = headingErrorToStartTurnInPlace.getValue();

         if (distance >= positionProximity && Math.abs(headingError) > errorToTurn)
         {
            isTurningToPath.set(true);
            turnInPlace(headingError);
            return;
         }
         isTurningToPath.set(false);
         phase.set(Phase.WALK);
         rateLimitedAngleToHeading.set(headingError);
      }

      if (phase.getValue() == Phase.TURN_TO_GOAL_YAW)
      {
         turnInPlace(AngleTools.computeAngleDifferenceMinusPiToPi(goalPose.getYaw(), currentPose.getYaw()));
         holdPositionWhileTurning();
         return;
      }

      walk(goalPose, distance);
   }

   /**
    * Heading to hold while walking. Turn-walk-turn faces along the start-to-goal line and keep-heading holds the heading
    * the waypoint started with. Otherwise the robot faces the goal position when far, and blends to the final heading
    * between {@code distanceToFaceGoal} and {@code distanceToMatchGoalAngle}.
    */
   private double computeWalkingHeadingYaw(FramePose3DReadOnly goalPose, double distance)
   {
      if (turnWalkTurn.getValue())
      {
         double lineX = goalPose.getX() - startPose.getX();
         double lineY = goalPose.getY() - startPose.getY();
         if (EuclidCoreTools.norm(lineX, lineY) > positionProximityOrSmall())
            return Math.atan2(lineY, lineX);
         return currentPose.getYaw();
      }

      if (keepHeading.getValue())
         return startPose.getYaw();

      double finalYaw = goalOrientationMatters.getValue() ? goalPose.getYaw() : currentPose.getYaw();
      // 1 = far from goal, face the goal position; 0 = near goal, match the final heading
      double faceGoalBlendFraction = MathTools.clamp((distance - distanceToMatchGoalAngle.getValue())
                                                     / (distanceToFaceGoal.getValue() - distanceToMatchGoalAngle.getValue()), 0.0, 1.0);
      if (faceGoalBlendFraction == 0.0)
         return finalYaw;
      double bearingToGoal = Math.atan2(vectorToGoal.getY(), vectorToGoal.getX());
      // Interpolate the angle, not the vectors: the vector to the goal is metres long and would dominate the blend.
      return AngleTools.trimAngleMinusPiToPi(finalYaw + faceGoalBlendFraction * AngleTools.computeAngleDifferenceMinusPiToPi(bearingToGoal, finalYaw));
   }

   private double positionProximityOrSmall()
   {
      return Math.max(distanceToGoalThresholdToStop.getValue(), 1.0e-3);
   }

   /** Turns toward {@code headingError} (positive is counter-clockwise) without translating. */
   private void turnInPlace(double headingError)
   {
      angleToHeading.set(headingError);
      rateLimitedAngleToHeading.set(headingError);
      // Keep turning until inside the orientation proximity, even when that is tighter than the deadband.
      double deadband = Math.min(angleDeadband.getValue(), 0.5 * angleToGoalThresholdToStop.getValue());
      double turningVelocity = clampTurningVelocity(kAngle.getValue() * DeadbandTools.applyDeadband(deadband, headingError));

      normalizedRadialSpeed.set(0.0);
      rateLimitedNormalizedRadialSpeed.set(0.0);
      desiredVelocity.setToZero();
      desiredSpeed.set(0.0);
      desiredLinearVelocity.setToZero();
      desiredAngularVelocity.set(0.0, 0.0, turningVelocity);
   }

   /**
    * The walking policy slides 0.2 to 0.3 m while turning in place, so step back toward the goal at a slow speed rather
    * than leaving the goal and walking back to it.
    */
   private void holdPositionWhileTurning()
   {
      desiredVelocity.set(vectorToGoalInPelvisFrame);
      desiredVelocity.scale(positionHoldGain.getValue());
      double speed = desiredVelocity.norm();
      if (speed > maxPositionHoldSpeed.getValue())
         desiredVelocity.scale(maxPositionHoldSpeed.getValue() / speed);
      desiredSpeed.set(desiredVelocity.norm());
      desiredLinearVelocity.set(desiredVelocity);
      currentPose.getOrientation().transform(desiredLinearVelocity);
   }

   private double clampTurningVelocity(double turningVelocity)
   {
      turningVelocity = MathTools.clamp(turningVelocity, maxTurningSpeed.getDoubleValue());
      if (Math.abs(turningVelocity) > 0.0 && Math.abs(turningVelocity) < minTurningSpeed.getValue())
         turningVelocity = Math.signum(turningVelocity) * minTurningSpeed.getValue();
      return turningVelocity;
   }

   private final Vector2D walkingTargetInBodyFrame = new Vector2D();

   private void walk(FramePose3DReadOnly goalPose, double distanceToGoal)
   {
      walkingHeadingYaw.set(computeWalkingHeadingYaw(goalPose, distanceToGoal));
      angleToHeading.set(AngleTools.computeAngleDifferenceMinusPiToPi(walkingHeadingYaw.getValue(), currentPose.getYaw()));
      // Rate-limit how quickly the heading target can change each tick
      double angleChangeFromPrevious = AngleTools.computeAngleDifferenceMinusPiToPi(angleToHeading.getDoubleValue(), rateLimitedAngleToHeading.getDoubleValue());
      angleChangeFromPrevious = MathTools.clamp(angleChangeFromPrevious, maxHeadingRate.getValue() * computedDt);
      rateLimitedAngleToHeading.set(AngleTools.trimAngleMinusPiToPi(rateLimitedAngleToHeading.getDoubleValue() + angleChangeFromPrevious));

      double distanceForMaxSpeed = 0.5 * MathTools.square(maxForwardSpeed.getDoubleValue()) / maxRadialAcceleration.getDoubleValue();
      // Acceleration limited controller on distance.
      if (distanceToGoal >= distanceForMaxSpeed)
      {
         normalizedRadialSpeed.set(1.0);
      }
      else
      {
         double speedForDeccel = Math.sqrt(2.0 * maxRadialAcceleration.getDoubleValue() * distanceToGoal);
         normalizedRadialSpeed.set(Math.max(speedForDeccel / maxForwardSpeed.getDoubleValue(), waypointThroughputSpeedScalar.getDoubleValue()));
      }
      // Acceleration-limit the ramp-up so speed increases gradually
      double maxAllowedSpeed = rateLimitedNormalizedRadialSpeed.getDoubleValue() + computedDt * maxRadialAcceleration.getValue();
      rateLimitedNormalizedRadialSpeed.set(MathTools.clamp(normalizedRadialSpeed.getValue(), 0.0, maxAllowedSpeed));

      // Walk toward the goal, or for turn-walk-turn toward a point ahead on the start-to-goal line.
      computeWalkingTarget(goalPose);
      double angleToTarget = Math.atan2(walkingTargetInBodyFrame.getY(), walkingTargetInBodyFrame.getX());
      double speed = rateLimitedNormalizedRadialSpeed.getDoubleValue() * desiredCruisingSpeedScalar.getValue();
      double xSpeedLimit = walkingTargetInBodyFrame.getX() > 0.0 ? maxForwardSpeed.getDoubleValue() : maxBackwardSpeed.getDoubleValue();
      // Fastest speed in this direction within the elliptical forward/lateral limits. Scaling each axis by its own
      // limit instead would bend the walking direction off the target.
      double cosine = Math.cos(angleToTarget);
      double sine = Math.sin(angleToTarget);
      double speedLimit = 1.0 / Math.sqrt(MathTools.square(cosine / xSpeedLimit) + MathTools.square(sine / maxLateralSpeed.getValue()));
      desiredVelocity.set(speed * speedLimit * cosine, speed * speedLimit * sine);

      // P-controller on heading. The walking policy tracks a gentle walking arc but locks the knees on a hard yaw with
      // little forward speed, so cap yaw to v / radius, or to a slow turn when not walking forward. Large heading
      // changes are made by the turn in place phases instead.
      double turningVelocity = clampTurningVelocity(kAngle.getValue() * DeadbandTools.applyDeadband(angleDeadband.getDoubleValue(),
                                                                                                      rateLimitedAngleToHeading.getDoubleValue()));
      double radius = Math.max(minTurnRadiusWhileWalking.getValue(), 1.0e-3);
      double maxTurn = Math.max(Math.max(0.0, desiredVelocity.getX()) / radius, maxTurningSpeedWhileWalkingSlowly.getValue());
      desiredAngularVelocity.set(0.0, 0.0, MathTools.clamp(turningVelocity, maxTurn));

      double desiredSpeed = desiredVelocity.norm();
      if (desiredSpeed < minSpeed.getValue() && desiredSpeed > 1e-3)
      {
         desiredVelocity.scale(minSpeed.getValue() / desiredSpeed);
         desiredSpeed = minSpeed.getDoubleValue();
      }
      this.desiredSpeed.set(desiredSpeed);

      desiredLinearVelocity.set(desiredVelocity);
      currentPose.getOrientation().transform(desiredLinearVelocity);
   }

   private final Vector2D walkingTarget = new Vector2D();

   private void computeWalkingTarget(FramePose3DReadOnly goalPose)
   {
      walkingTarget.set(goalPose.getX(), goalPose.getY());
      if (turnWalkTurn.getValue())
      {
         double lineX = goalPose.getX() - startPose.getX();
         double lineY = goalPose.getY() - startPose.getY();
         double lineLength = EuclidCoreTools.norm(lineX, lineY);
         if (lineLength > positionProximityOrSmall())
         {
            lineX /= lineLength;
            lineY /= lineLength;
            double distanceAlongLine = (currentPose.getX() - startPose.getX()) * lineX + (currentPose.getY() - startPose.getY()) * lineY;
            double lookaheadAlongLine = Math.max(distanceAlongLine, 0.0) + pathLookaheadDistance.getValue();
            if (lookaheadAlongLine < lineLength)
               walkingTarget.set(startPose.getX() + lookaheadAlongLine * lineX, startPose.getY() + lookaheadAlongLine * lineY);
         }
      }
      walkingTarget.sub(currentPose.getX(), currentPose.getY());
      currentPose.getOrientation().inverseTransform(walkingTarget, walkingTargetInBodyFrame);
   }

   private void updateOutputMessage()
   {
      outputMessage.setForwardVelocity(desiredVelocity.getX());
      outputMessage.setLateralVelocity(desiredVelocity.getY());
      outputMessage.setTurnVelocity(desiredAngularVelocity.getZ());
      outputMessage.setWalk(!hasReachedGoal.getBooleanValue());
   }

   private static class GoalWaypoint
   {
      private final FramePose2D goalPose = new FramePose2D();
      private double proximityToReachGoal;
      private double angleToReachGoal;
      private double cruisingSpeedScaler;
      private boolean goalOrientationMatters;
      private boolean keepHeading;
      private boolean turnWalkTurn;
      /** Unit vector of the leg that arrives at this waypoint (from previousPose to goalPose). */
      private final Vector2D incomingDirection = new Vector2D();

      public void set(ControllerWaypointGoalCommand command, FramePose2DReadOnly previousPose, double maxSpeed)
      {
         goalPose.set(command.getGoalPose());
         angleToReachGoal = command.getOrientationProximity();
         proximityToReachGoal = command.getPositionProximity();
         goalOrientationMatters = command.getGoalOrientationMatters();
         keepHeading = command.getKeepHeading();
         turnWalkTurn = command.getTurnWalkTurn();

         double dx = goalPose.getX() - previousPose.getX();
         double dy = goalPose.getY() - previousPose.getY();
         double distance = EuclidCoreTools.norm(dx, dy);
         if (distance > 1e-6)
         {
            incomingDirection.set(dx, dy);
            incomingDirection.scale(1.0 / distance);
         }
         else
         {
            incomingDirection.set(1.0, 0.0);
         }

         if (command.getTimeToReachGoal() > 0.0)
         {
            double speed = distance / command.getTimeToReachGoal();
            cruisingSpeedScaler = Math.min(1.0, speed / maxSpeed);
         }
         else
         {
            cruisingSpeedScaler = 1.0;
         }
      }

      public FramePose2DReadOnly getGoalPose()
      {
         return goalPose;
      }

      public double getProximityToReachGoal()
      {
         return proximityToReachGoal;
      }

      public double getAngleToReachGoal()
      {
         return angleToReachGoal;
      }

      public double getCruisingSpeedScaler()
      {
         return cruisingSpeedScaler;
      }

      public boolean getGoalOrientationMatters()
      {
         return goalOrientationMatters;
      }

      public boolean getKeepHeading()
      {
         return keepHeading;
      }

      public boolean getTurnWalkTurn()
      {
         return turnWalkTurn;
      }

      public Vector2DReadOnly getIncomingDirection()
      {
         return incomingDirection;
      }
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(newYoGraphicArrow3D("Goal Linear Velocity", basePosition, desiredLinearVelocity, 1.0, ColorDefinitions.Yellow()));
      group.addChild(newYoGraphicArrow3D("Goal Angular Velocity", basePosition, desiredAngularVelocity, 1.0, ColorDefinitions.Orange()));
      group.addChild(newYoGraphicPoint3D("Current Goal Position", currentGoalPose.getPosition(), 0.1, ColorDefinitions.Red()));
      group.addChild(newYoGraphicArrow3D("Current Goal Heading", currentGoalPose.getPosition(), currentGoalDirection, 0.5, ColorDefinitions.Red()));

      group.addChild(newYoGraphicPoint3D("Current Position", currentPose.getPosition(), 0.1, ColorDefinitions.Orange()));
      group.addChild(newYoGraphicArrow3D("Current Heading", currentPose.getPosition(), currentDirection, 0.5, ColorDefinitions.Orange()));

      group.addChild(newYoGraphicPoint3D("Start Position", startPose.getPosition(), 0.1, ColorDefinitions.Green()));
      group.addChild(newYoGraphicArrow3D("Start Heading", startPose.getPosition(), startDirection, 0.5, ColorDefinitions.Green()));
      group.addChild(newYoGraphicArrow3D("Start To Goal Path", startPose.getPosition(), startToGoalVector,
                                         true, 1.0, 0.0, false, 0.01, 0.0, ColorDefinitions.Green()));

      group.addChild(waypointVisualizer.getSCS2YoGraphics());

      return group;
   }
}
