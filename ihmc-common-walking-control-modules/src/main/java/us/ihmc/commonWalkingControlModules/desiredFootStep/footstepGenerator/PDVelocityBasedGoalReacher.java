package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import controller_msgs.msg.dds.ControllerWalkToGoalStatusMessage;
import controller_msgs.msg.dds.ControllerWaypointStatusMessage;
import controller_msgs.msg.dds.DirectionalControlInputMessage;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ControllerReleaseGoalCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ControllerWaypointGoalCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ControllerWaypointGoalListCommand;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
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

import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicArrow3D;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.newYoGraphicPoint3D;

/**
 * Class uses a clamped PD controller that aligns headings to reach a goal.
 */
public class PDVelocityBasedGoalReacher implements Updatable, SCS2YoGraphicHolder
{
   private static final Vector2DReadOnly forwardVector = new Vector2D(1.0, 0.0);

   private static final double BASE_POSITION_Z_OFFSET_FOR_VISUALIZATION = 0.75;
   private static final double GOAL_POSITION_Z_OFFSET_FOR_VISUALIZATION = 0.1;

   // TODO extract these in to a parameter file.
   private static final double DEFAULT_MAX_RADIAL_ACCELERATION = 1.5;
   private static final double DEFAULT_MAX_HEADING_RATE = 2.0 * Math.PI / 3.0;

   private static final double DEFAULT_MAX_FORWARD_SPEED = 0.7;
   private static final double DEFAULT_MAX_BACKWARD_SPEED = 0.4;
   private static final double DEFAULT_MAX_LATERAL_SPEED = 0.6;

   private static final double DEFAULT_K_RADIUS = 2.0;
   private static final double DEFAULT_K_DELTA = 1.5;

   private static final double DEFAULT_DISTANCE_TO_GOAL_THRESHOLD_TO_STOP = 0.02;
   private static final double DEFAULT_ANGLE_TO_GOAL_THRESHOLD_TO_STOP = Math.toRadians(5.0);

   private static final double DEFAULT_DISTANCE_TO_MATCH_GOAL_ANGLE = 0.5;
   private static final double DEFAULT_DISTANCE_TO_FACE_GOAL = 1.5;

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

   private final YoDouble maxForwardSpeed = new YoDouble("maxForwardSpeed", registry);
   private final YoDouble maxLateralSpeed = new YoDouble("maxLateralSpeed", registry);
   private final YoDouble maxBackwardSpeed = new YoDouble("maxBackwardSpeed", registry);

   private final YoDouble kRadius = new YoDouble("kRadius", registry);
   private final YoDouble kDelta = new YoDouble("kDelta", registry);
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
   private final YoBoolean justReachedGoal = new YoBoolean("justReachedGoal", registry);
   private final YoBoolean goalOrientationMatters = new YoBoolean("goalOrientationMatters", registry);

   // Feedback terms
   private final YoDouble normalizedRadialSpeed;
   private final YoDouble rateLimitedNormalizedRadialSpeed;
   // CLF terms
   private final YoFramePoint3D basePosition = new YoFramePoint3D("basePosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoVector2D desiredVelocity = new YoVector2D("desiredVelocity", registry);
   private final YoDouble desiredSpeed = new YoDouble("desiredSpeed", registry);
   private final YoFrameVector3D desiredLinearVelocity = new YoFrameVector3D("desiredLinearVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D desiredAngularVelocity = new YoFrameVector3D("desiredAngularVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final DirectionalControlInputMessage outputMessage = new DirectionalControlInputMessage();

   // Temp Variables
   private final Vector2D finalHeadingInBodyFrame = new Vector2D();     // goal's final heading direction, in robot body frame
   private final Vector2D desiredHeadingInBodyFrame = new Vector2D();   // blended desired heading, in robot body frame
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
      kRadius.set(DEFAULT_K_RADIUS);
      kDelta.set(DEFAULT_K_DELTA);

      distanceToGoalThresholdToStop.set(DEFAULT_DISTANCE_TO_GOAL_THRESHOLD_TO_STOP);
      angleToGoalThresholdToStop.set(DEFAULT_ANGLE_TO_GOAL_THRESHOLD_TO_STOP);

      distanceToMatchGoalAngle.set(DEFAULT_DISTANCE_TO_MATCH_GOAL_ANGLE);
      distanceToFaceGoal.set(DEFAULT_DISTANCE_TO_FACE_GOAL);

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
      }
   }

   private final ControllerWalkToGoalStatusMessage statusMessage = new ControllerWalkToGoalStatusMessage();
   private final ControllerWaypointStatusMessage waypointReachedMessage = new ControllerWaypointStatusMessage();

   @Override
   public void update(double time)
   {
      currentPose.setFromReferenceFrame(frameOfTheBase);

      basePosition.set(currentPose.getPosition());
      basePosition.addZ(BASE_POSITION_Z_OFFSET_FOR_VISUALIZATION);

      // Snapshot state before updateGoalPose() so we can detect transitions (e.g. goal just reached)
      // and send a status update even when the goal changes mid-tick.
      boolean hasGoal = this.hasGoal.getBooleanValue();
      boolean hasReachedGoal = this.hasReachedGoal.getBooleanValue();
      if (Double.isNaN(lastTickTime))
         computedDt = 0.0;
      else
         computedDt = time - lastTickTime;
      lastTickTime = time;

      updateGoalPose();
      waypointVisualizer.update(goalPoses.size(), i -> goalPoses.get(i).getGoalPose());

      // Detect the false→true transition so we can emit a stop command exactly once.
      justReachedGoal.set(!hasReachedGoal && this.hasReachedGoal.getBooleanValue());

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

         updateHeadingControl(currentGoalPose);
         computePDNormalizedFeedbackVelocities(distanceToGoal.getValue(), rateLimitedAngleToHeading.getDoubleValue());
         updateOutputMessage();
      }
      else if (justReachedGoal.getBooleanValue())
      {
         // Goal reached this tick: send one stop command to halt the robot.
         outputMessage.setForward(0.0);
         outputMessage.setRight(0.0);
         outputMessage.setClockwise(0.0);
         outputMessage.setWalk(false);
      }
      else if (!this.hasGoal.getBooleanValue())
      {
         startDirection.setToNaN();
         startToGoalVector.setToNaN();
      }
      // else: holding at goal on a subsequent tick — stop command already sent, nothing to do.

      if (hasGoal || this.hasGoal.getBooleanValue() || (hasReachedGoal != this.hasReachedGoal.getValue()))
      {
         // Send status.
         statusMessage.getCurrentPosition().set(currentPose.getPosition());
         statusMessage.setCurrentGoalXPosition(currentGoalPose.getX());
         statusMessage.setCurrentGoalYPosition(currentGoalPose.getY());
         statusMessage.setCurrentGoalYaw(currentGoalPose.getYaw());
         statusMessage.setIsReached(this.hasReachedGoal.getBooleanValue());
         reportWalkToGoalStatus();
      }
   }

   private void reportWalkToGoalStatus()
   {
      statusMessageOutputManager.reportStatusMessage(statusMessage);
   }

   public DirectionalControlInputMessage getOutputMessage()
   {
      // Pass through the stop command on the exact tick the goal is reached.
      if (justReachedGoal.getBooleanValue())
         return outputMessage;

      // Normal walking: deliver the command only while actively pursuing a goal.
      if (!hasGoal.getBooleanValue() || hasReachedGoal.getBooleanValue())
         return null;

      return outputMessage;
   }

   private void updateGoalPose()
   {
      // If we have no goal poses in queue, don't do anything.
      if (goalPoses.isEmpty())
      {
         hasGoal.set(false);
         currentGoalPose.setToNaN();
         hasReachedGoal.set(true);
         return;
      }

      // Advance through any waypoints that have already been reached, stopping at the first unmet one.
      // After the loop, reachedGoal reflects whether the current front waypoint is met.
      boolean reachedGoal = true;
      while (!goalPoses.isEmpty() && reachedGoal)
      {
         reachedGoal = reachedGoal(goalPoses.getFirst().getGoalPose());
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
               // Completed the final waypoint and we're not holding; discard it and stop.
               publishWaypointStatus(goalPoses.getFirst().getGoalPose(), goalPoses.size() - 1, false);
               goalPoses.remove(0);
               wasGoalReached.set(false);
            }
            else if (shouldHoldGoal.getBooleanValue())
            {
               // Holding at the final waypoint; keep it in the list so the robot stays.
               break;
            }
         }
      }

      hasGoal.set(!goalPoses.isEmpty());

      // Publish "waypoint reached" exactly once when the final (held) goal is first met.
      // wasGoalReached persists across ticks so we don't re-publish every tick while holding.
      if (reachedGoal && !wasGoalReached.getBooleanValue() && !goalPoses.isEmpty())
      {
         publishWaypointStatus(goalPoses.getFirst().getGoalPose(), goalPoses.size() - 1, false);
      }

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
      }
      else
      {
         currentGoalPose.setToNaN();
      }
   }

   private void captureStartPose()
   {
      startPose.setFromReferenceFrame(frameOfTheBase);
      startPose.getPosition().setZ(GOAL_POSITION_Z_OFFSET_FOR_VISUALIZATION);
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

   private boolean reachedGoal(FramePose2DReadOnly currentGoalPose)
   {
      distanceToGoal.set(currentPose.getPosition().distanceXY(currentGoalPose.getPosition()));
      angleToGoal.set(AngleTools.computeAngleDifferenceMinusPiToPi(currentPose.getYaw(), currentGoalPose.getYaw()));

      return distanceToGoal.getDoubleValue() < distanceToGoalThresholdToStop.getValue() && Math.abs(angleToGoal.getValue()) < angleToGoalThresholdToStop.getValue();
   }

   /**
    * Computes the heading error and rate-limits the heading target used by the PD feedback.
    * <p>
    * Far from the goal the robot faces toward the goal position; close to the goal it
    * transitions to matching the goal's final orientation. The blend is governed by
    * {@code distanceToFaceGoal} (start facing goal) and {@code distanceToMatchGoalAngle} (fully
    * aligned with final heading).
    * </p>
    */
   private void updateHeadingControl(FramePose3DReadOnly goalPose)
   {
      // Vector from the robot to the goal, expressed in the robot's body frame
      vectorToGoal.set(goalPose.getPosition());
      vectorToGoal.sub(currentPose.getX(), currentPose.getY());
      currentPose.getOrientation().inverseTransform(vectorToGoal, vectorToGoalInPelvisFrame);

      if (goalOrientationMatters.getValue())
      {
         // Goal's final heading direction rotated into the robot's body frame
         finalHeadingInBodyFrame.set(currentGoalDirection);
         currentPose.getOrientation().inverseTransform(finalHeadingInBodyFrame, finalHeadingInBodyFrame);

         // 1 = far from goal → face the goal position; 0 = near goal → match the goal's final heading
         double faceGoalBlendFraction = MathTools.clamp(
               (vectorToGoal.norm() - distanceToMatchGoalAngle.getValue()) / (distanceToFaceGoal.getDoubleValue() - distanceToMatchGoalAngle.getDoubleValue()),
               0.0, 1.0);

         // Blend between the goal's final heading (alpha=0) and the direction toward the goal (alpha=1)
         desiredHeadingInBodyFrame.interpolate(finalHeadingInBodyFrame, vectorToGoalInPelvisFrame, faceGoalBlendFraction);
      }
      else
      {
         // Orientation doesn't matter: always face toward the goal position
         desiredHeadingInBodyFrame.set(vectorToGoalInPelvisFrame);
      }

      // Signed angle error: how far the robot must rotate to face the desired heading (forward = [1, 0])
      angleToHeading.set(AngleTools.angleMinusPiToPi(desiredHeadingInBodyFrame, forwardVector));

      // Rate-limit how quickly the heading target can change each tick
      double angleChangeFromPrevious = AngleTools.computeAngleDifferenceMinusPiToPi(angleToHeading.getDoubleValue(), rateLimitedAngleToHeading.getDoubleValue());
      angleChangeFromPrevious = MathTools.clamp(angleChangeFromPrevious, maxHeadingRate.getValue() * computedDt);
      rateLimitedAngleToHeading.set(AngleTools.trimAngleMinusPiToPi(angleToHeading.getDoubleValue() + angleChangeFromPrevious));
   }


   private void computePDNormalizedFeedbackVelocities(double distanceToGoal, double angleToGoalHeading)
   {
      // P-controller on distance: normalize forward speed to [0, 1]
      normalizedRadialSpeed.set(MathTools.clamp(kRadius.getValue() * distanceToGoal, 0.0, 1.0));
      // Acceleration-limit the ramp-up so speed increases gradually
      double maxAllowedSpeed = rateLimitedNormalizedRadialSpeed.getDoubleValue() + computedDt * maxRadialAcceleration.getValue();
      rateLimitedNormalizedRadialSpeed.set(MathTools.clamp(normalizedRadialSpeed.getValue(), 0.0, maxAllowedSpeed));

      // P-controller on heading: angular velocity proportional to heading error
      double turningVelocity = -kDelta.getValue() * angleToGoalHeading;
      desiredAngularVelocity.set(0.0, 0.0, turningVelocity);

      // Angle from the robot's forward axis to the goal direction, measured in the robot's body frame
      double angleToGoalInBodyFrame = AngleTools.angleMinusPiToPi(vectorToGoalInPelvisFrame, forwardVector);

      // Unit components of the velocity direction toward the goal, scaled by normalized speed
      double speed = rateLimitedNormalizedRadialSpeed.getDoubleValue() * desiredCruisingSpeedScalar.getValue();
      double goalDirectionX = speed * Math.cos(-angleToGoalInBodyFrame);
      double goalDirectionY = speed * Math.sin(-angleToGoalInBodyFrame);

      // Scale by per-axis speed limits; forward and backward limits are asymmetric
      double xSpeedLimit = vectorToGoalInPelvisFrame.getX() > 0 ? maxForwardSpeed.getDoubleValue() : maxBackwardSpeed.getDoubleValue();
      double vx = goalDirectionX * xSpeedLimit;
      double vy = goalDirectionY * maxLateralSpeed.getValue();

      desiredVelocity.set(vx, vy);
      desiredSpeed.set(desiredVelocity.norm());

      desiredLinearVelocity.set(desiredVelocity);
      currentPose.getOrientation().transform(desiredLinearVelocity);
   }

   private void updateOutputMessage()
   {
      outputMessage.setForward(desiredVelocity.getX());
      outputMessage.setRight(-desiredVelocity.getY());
      outputMessage.setClockwise(-desiredAngularVelocity.getZ());
      outputMessage.setWalk(!hasReachedGoal.getBooleanValue());
   }

   private static class GoalWaypoint
   {
      private final FramePose2D goalPose = new FramePose2D();
      private double proximityToReachGoal;
      private double angleToReachGoal;
      private double cruisingSpeedScaler;
      private boolean goalOrientationMatters;

      public void set(ControllerWaypointGoalCommand command, FramePose2DReadOnly previousPose, double maxSpeed)
      {
         goalPose.set(command.getGoalPose());
         angleToReachGoal = command.getOrientationProximity();
         proximityToReachGoal = command.getPositionProximity();
         goalOrientationMatters = command.getGoalOrientationMatters();

         double distance = command.getGoalPose().getPositionDistance(previousPose.getPosition());
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
