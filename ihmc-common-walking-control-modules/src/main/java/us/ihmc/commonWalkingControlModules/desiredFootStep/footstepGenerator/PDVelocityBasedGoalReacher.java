package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import controller_msgs.msg.dds.ControllerWalkToGoalStatusMessage;
import controller_msgs.msg.dds.DirectionalControlInputMessage;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ControllerReleaseGoalCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ControllerWalkToGoalCommand;
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
import us.ihmc.yoVariables.providers.DoubleProvider;
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


   private final RecyclingArrayList<FramePose2D> goalPoses = new RecyclingArrayList<>(FramePose2D::new);
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoFramePose3D currentGoalPose = new YoFramePose3D("goalPose", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D currentGoalDirection = new YoFrameVector3D("goalDirection", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose3D currentPose = new YoFramePose3D("currentPose", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D currentDirection = new YoFrameVector3D("currentDirection", ReferenceFrame.getWorldFrame(), registry);

   // Frame of the root link to track to the goal
   private final ReferenceFrame frameOfTheBase;

   // control parameters
   private double lastTickTime = Double.NaN;
   private final YoDouble maxRadialAcceleration = new YoDouble("maxRadialAcceleration", registry);
   private final YoDouble maxHeadingRate = new YoDouble("maxHeadingRate", registry);
   private double computedDt;

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
   private final YoDouble limitedAngleToHeading = new YoDouble("limitedAngleToHeading", registry);

   private final YoBoolean hasGoal = new YoBoolean("hasGoal", registry);
   private final YoBoolean shouldHoldGoal = new YoBoolean("shouldHoldGoal", registry);
   private final YoBoolean hasReachedGoal = new YoBoolean("hasReachedGoal", registry);

   // Feedback terms
   private final YoDouble normalizedRadialSpeed;
   private final YoDouble limitedNormalizedRadialSpeed;
   // CLF terms
   private final YoFramePoint3D basePosition = new YoFramePoint3D("basePosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoVector2D desiredVelocity = new YoVector2D("desiredVelocity", registry);
   private final YoDouble desiredSpeed = new YoDouble("desiredSpeed", registry);
   private final YoFrameVector3D desiredLinearVelocity = new YoFrameVector3D("desiredLinearVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D desiredAngularVelocity = new YoFrameVector3D("desiredAngularVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final DirectionalControlInputMessage outputMessage = new DirectionalControlInputMessage();

   // Temp Variables
   private final Vector2D terminalHeadingInPelvisFrame = new Vector2D();
   private final Vector2D currentHeadingInPelvisFrame = new Vector2D();
   private final Vector2D objectiveHeadingInPelvisFrame = new Vector2D();
   private final Vector2D vectorToGoal = new Vector2D();

   private final StatusMessageOutputManager statusMessageOutputManager;

   public PDVelocityBasedGoalReacher(ReferenceFrame frameOfBase,
                                     StatusMessageOutputManager statusMessageOutputManager,
                                     YoRegistry parentRegistry)
   {
      this.frameOfTheBase = frameOfBase;
      this.statusMessageOutputManager = statusMessageOutputManager;

      normalizedRadialSpeed = new YoDouble("normalizedRadialSpeed", registry);
      limitedNormalizedRadialSpeed = new YoDouble("limitedNormalizedRadialSpeed", registry);

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
   }

   public void consumeNewWaypoint(ControllerWalkToGoalCommand command)
   {
      goalPoses.add().set(currentGoalPose);
      hasGoal.set(true);
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

   private void setCurrentGoalPoseInternal(FramePose2DReadOnly currentGoalPose)
   {

   }

   private final ControllerWalkToGoalStatusMessage statusMessage = new ControllerWalkToGoalStatusMessage();

   @Override
   public void update(double time)
   {
      currentPose.setFromReferenceFrame(frameOfTheBase);

      basePosition.set(currentPose.getPosition());
      basePosition.addZ(BASE_POSITION_Z_OFFSET_FOR_VISUALIZATION);

      // We want to hold onto this from the beginning, so that if we reach it we can publish the status.
      boolean hasGoal = this.hasGoal.getBooleanValue();
      if (Double.isNaN(lastTickTime))
         computedDt = 0.0;
      else
         computedDt = time - lastTickTime;
      lastTickTime = time;

      updateGoalPose();

      if (this.hasGoal.getBooleanValue())
      {
         // Update this value for visualization.
         currentGoalDirection.set(1.0, 0.0, 0.0);
         currentGoalPose.getOrientation().transform(currentGoalDirection);

         currentPose.getPosition().setZ(GOAL_POSITION_Z_OFFSET_FOR_VISUALIZATION);
         currentDirection.set(1.0, 0.0, 0.0);
         currentPose.getOrientation().transform(currentDirection);

         updateDistanceAndAngleErrorForFeedback(currentGoalPose);
         computePDNormalizedFeedbackVelocities(distanceToGoal.getValue(), limitedAngleToHeading.getDoubleValue());
         updateOutputMessage();
      }

      if (hasGoal || this.hasGoal.getBooleanValue())
      {
         // Send status.
         statusMessage.getCurrentPosition().set(currentPose.getPosition());
         statusMessage.setCurrentGoalXPosition(currentGoalPose.getX());
         statusMessage.setCurrentGoalYPosition(currentGoalPose.getY());
         statusMessage.setCurrentGoalYaw(currentGoalPose.getYaw());
         statusMessage.setIsReached(hasReachedGoal.getBooleanValue());
      }

      statusMessageOutputManager.reportStatusMessage(statusMessage);
   }

   public DirectionalControlInputMessage getOutputMessage()
   {
      if (hasReachedGoal.getBooleanValue())
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

      // Check to find the first goal not reached.
      boolean reachedGoal = true;
      while (!goalPoses.isEmpty() && reachedGoal)
      {
         reachedGoal = reachedGoal(goalPoses.getFirst());
         if (reachedGoal)
         {
            if (goalPoses.size() > 1)
               goalPoses.remove(0);
            else if (goalPoses.size() == 1 && !shouldHoldGoal.getBooleanValue())
               goalPoses.remove(0);
            else if (shouldHoldGoal.getBooleanValue())
               break;
         }
      }

      hasGoal.set(!goalPoses.isEmpty());
      hasReachedGoal.set(reachedGoal);
      if (goalPoses.size() > 0)
      {
         this.currentGoalPose.checkReferenceFrameMatch(goalPoses.getFirst().getReferenceFrame());
         this.currentGoalPose.getPosition().set(goalPoses.getFirst().getPosition(), GOAL_POSITION_Z_OFFSET_FOR_VISUALIZATION);
         this.currentGoalPose.getOrientation().setToYawOrientation(goalPoses.getFirst().getYaw());
      }
      else
      {
         currentGoalPose.setToNaN();
      }
   }

   private boolean reachedGoal(FramePose2DReadOnly currentGoalPose)
   {
      distanceToGoal.set(currentPose.getPosition().distanceXY(currentGoalPose.getPosition()));
      angleToGoal.set(AngleTools.computeAngleDifferenceMinusPiToPi(currentPose.getYaw(), currentGoalPose.getYaw()));

      return distanceToGoal.getDoubleValue() < distanceToGoalThresholdToStop.getValue() && Math.abs(angleToGoal.getValue()) < angleToGoalThresholdToStop.getValue();
   }

   private void updateDistanceAndAngleErrorForFeedback(FramePose3DReadOnly currentGoalPose)
   {
      // This is the heading to the goal
      vectorToGoal.set(currentGoalPose.getPosition());
      vectorToGoal.sub(currentPose.getX(), currentPose.getY());
      currentPose.getOrientation().inverseTransform(vectorToGoal, vectorToGoalInPelvisFrame);

      // This is the heading at the goal
      terminalHeadingInPelvisFrame.set(currentGoalDirection);
      currentPose.getOrientation().inverseTransform(terminalHeadingInPelvisFrame, terminalHeadingInPelvisFrame);

      double turnAlpha = MathTools.clamp((vectorToGoal.norm() - distanceToMatchGoalAngle.getValue()) / (distanceToFaceGoal.getDoubleValue()
                                                                                                              - distanceToMatchGoalAngle.getDoubleValue()),
                                         0.0,
                                         1.0);

      // Compute the desired heading, blending between what we want to reach at the end and going straight toward the goal
      objectiveHeadingInPelvisFrame.interpolate(terminalHeadingInPelvisFrame, vectorToGoalInPelvisFrame, turnAlpha);

      currentPose.getOrientation().transform(forwardVector, currentHeadingInPelvisFrame);

      angleToHeading.set(AngleTools.angleMinusPiToPi(objectiveHeadingInPelvisFrame, forwardVector));
      double angleChangeFromPrevious = AngleTools.computeAngleDifferenceMinusPiToPi(angleToHeading.getDoubleValue(), limitedAngleToHeading.getDoubleValue());
      angleChangeFromPrevious = MathTools.clamp(angleChangeFromPrevious, maxHeadingRate.getValue() * computedDt);
      limitedAngleToHeading.set(AngleTools.trimAngleMinusPiToPi(angleToHeading.getDoubleValue() + angleChangeFromPrevious));
   }


   private void computePDNormalizedFeedbackVelocities(double distanceToGoal, double angleToGoalHeading)
   {
      normalizedRadialSpeed.set(MathTools.clamp(kRadius.getValue() * distanceToGoal, 0.0, 1.0));
      double maxSpeed = limitedNormalizedRadialSpeed.getDoubleValue() + computedDt * maxRadialAcceleration.getValue();
      limitedNormalizedRadialSpeed.set(MathTools.clamp(normalizedRadialSpeed.getValue(), 0.0, maxSpeed));

      double turningVelocity = -kDelta.getValue() * angleToGoalHeading;
      desiredAngularVelocity.set(0.0, 0.0, turningVelocity);

      // Get the angle the desired direction is pointing
      double localHeadingAngle = AngleTools.angleMinusPiToPi(vectorToGoalInPelvisFrame, forwardVector);

      double cosDelta = limitedNormalizedRadialSpeed.getDoubleValue() * Math.cos(-localHeadingAngle);
      double sinDelta = limitedNormalizedRadialSpeed.getDoubleValue() * Math.sin(-localHeadingAngle);

      double xSpeed = vectorToGoalInPelvisFrame.getX() > 0 ? maxForwardSpeed.getDoubleValue() : maxBackwardSpeed.getDoubleValue();
      double vx = cosDelta * xSpeed;
      double vy = sinDelta * maxLateralSpeed.getValue();

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

      return group;
   }
}
