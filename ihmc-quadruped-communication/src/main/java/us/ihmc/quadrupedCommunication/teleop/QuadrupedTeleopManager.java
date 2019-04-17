package us.ihmc.quadrupedCommunication.teleop;

import com.google.common.util.concurrent.AtomicDouble;
import controller_msgs.msg.dds.*;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.quadrupedBasics.QuadrupedSteppingRequestedEvent;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedOrientedStep;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsBasics;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedPlanning.stepStream.bodyPath.QuadrupedBodyPathMultiplexer;
import us.ihmc.quadrupedPlanning.footstepChooser.PlanarGroundPointFootSnapper;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapper;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitStepStream;
import us.ihmc.robotics.math.filters.RateLimitedYoFrameVector;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedTeleopManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final QuadrupedXGaitStepStream stepStream;
   private final YoQuadrupedXGaitSettings xGaitSettings;
   private final YoDouble timestamp = new YoDouble("timestamp", registry);
   private final YoBoolean walking = new YoBoolean("walking", registry);
   private final Ros2Node ros2Node;

   private final YoBoolean xGaitRequested = new YoBoolean("xGaitRequested", registry);
   private final YoFrameVector3D desiredVelocity = new YoFrameVector3D("teleopDesiredVelocity", worldFrame, registry);
   private final YoDouble desiredVelocityRateLimit = new YoDouble("teleopDesiredVelocityRateLimit", registry);
   private final YoEnum<HighLevelControllerName> controllerRequestedEvent = new YoEnum<>("teleopControllerRequestedEvent", registry,
                                                                                         HighLevelControllerName.class, true);
   private final RateLimitedYoFrameVector limitedDesiredVelocity;

   private final YoBoolean standingRequested = new YoBoolean("standingRequested", registry);
   private final YoDouble firstStepDelay = new YoDouble("firstStepDelay", registry);
   private final AtomicBoolean paused = new AtomicBoolean(false);
   private final AtomicDouble desiredBodyHeight = new AtomicDouble();
   private final AtomicDouble desiredPositionX = new AtomicDouble();
   private final AtomicDouble desiredPositionY = new AtomicDouble();
   private final AtomicDouble desiredOrientationYaw = new AtomicDouble();
   private final AtomicDouble desiredOrientationPitch = new AtomicDouble();
   private final AtomicDouble desiredOrientationRoll = new AtomicDouble();
   private final AtomicDouble desiredOrientationTime = new AtomicDouble();

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();
   private final AtomicLong timestampNanos = new AtomicLong();

   private final QuadrupedBodyOrientationMessage offsetBodyOrientationMessage = new QuadrupedBodyOrientationMessage();
   private final QuadrupedBodyTrajectoryMessage bodyTrajectoryMessage = new QuadrupedBodyTrajectoryMessage();
   private final QuadrupedReferenceFrames referenceFrames;
   private final ReferenceFrame centerFeetZUpFrame;
   private final QuadrupedBodyPathMultiplexer bodyPathMultiplexer;
   private final IHMCROS2Publisher<HighLevelStateMessage> controllerStatePublisher;
   private final IHMCROS2Publisher<QuadrupedRequestedSteppingStateMessage> steppingStatePublisher;
   private IHMCROS2Publisher<QuadrupedTimedStepListMessage> timedStepListPublisher;
   private IHMCROS2Publisher<QuadrupedBodyOrientationMessage> bodyOrientationPublisher;
   private IHMCROS2Publisher<QuadrupedBodyTrajectoryMessage> bodyTrajectoryPublisher;
   private IHMCROS2Publisher<QuadrupedBodyHeightMessage> bodyHeightPublisher;

   public QuadrupedTeleopManager(String robotName, Ros2Node ros2Node, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, double initialBodyHeight,
                                 QuadrupedReferenceFrames referenceFrames, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this(robotName, ros2Node, defaultXGaitSettings, initialBodyHeight, referenceFrames, 0.01, graphicsListRegistry, parentRegistry);
   }

   public QuadrupedTeleopManager(String robotName, Ros2Node ros2Node, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, double initialBodyHeight,
                                 QuadrupedReferenceFrames referenceFrames, double updateDT, YoGraphicsListRegistry graphicsListRegistry,
                                 YoVariableRegistry parentRegistry)
   {
      this.referenceFrames = referenceFrames;
      this.ros2Node = ros2Node;
      this.xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);
      centerFeetZUpFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();

      firstStepDelay.set(0.5);
      this.bodyPathMultiplexer = new QuadrupedBodyPathMultiplexer(referenceFrames, timestamp, xGaitSettings, firstStepDelay, graphicsListRegistry, registry);
      this.stepStream = new QuadrupedXGaitStepStream(xGaitSettings, timestamp, bodyPathMultiplexer, firstStepDelay, registry);

      desiredVelocityRateLimit.set(10.0);
      limitedDesiredVelocity = new RateLimitedYoFrameVector("limitedTeleopDesiredVelocity", "", registry, desiredVelocityRateLimit, updateDT, desiredVelocity);

      controllerRequestedEvent.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            HighLevelControllerName requestedState = controllerRequestedEvent.getEnumValue();
            if (requestedState != null)
            {
               controllerRequestedEvent.set(null);
               HighLevelStateMessage controllerMessage = new HighLevelStateMessage();
               controllerMessage.setHighLevelControllerName(requestedState.toByte());
               controllerStatePublisher.publish(controllerMessage);
            }
         }
      });

      desiredBodyHeight.set(initialBodyHeight);
      PlanarGroundPointFootSnapper snapper = new PlanarGroundPointFootSnapper(referenceFrames);
      stepStream.setStepSnapper(snapper);

      MessageTopicNameGenerator controllerPubGenerator = QuadrupedControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedGroundPlaneMessage.class, controllerPubGenerator, s -> snapper.submitGroundPlane(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, HighLevelStateChangeStatusMessage.class, controllerPubGenerator,
                                           s -> controllerStateChangeMessage.set(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedSteppingStateChangeMessage.class, controllerPubGenerator,
                                           s -> steppingStateChangeMessage.set(s.takeNextData()));
      ROS2Tools
            .createCallbackSubscription(ros2Node, RobotConfigurationData.class, controllerPubGenerator, s -> timestampNanos.set(s.takeNextData().timestamp_));
      ROS2Tools.createCallbackSubscription(ros2Node, HighLevelStateMessage.class, controllerPubGenerator, s -> paused.set(true));

      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedBodyPathPlanMessage.class, controllerPubGenerator,
                                           s -> bodyPathMultiplexer.handleBodyPathPlanMessage(s.takeNextData()));

      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedFootstepStatusMessage.class, controllerPubGenerator, s -> {
         QuadrupedFootstepStatusMessage packet = s.takeNextData();
         if (packet.getFootstepStatus() == QuadrupedFootstepStatusMessage.FOOTSTEP_STATUS_STARTED)
         {
            RobotQuadrant quadrant = RobotQuadrant.fromByte((byte) packet.getFootstepQuadrant());
            bodyPathMultiplexer.startedFootstep(quadrant, packet);
         }
      });

      ROS2Tools.createCallbackSubscription(ros2Node, QuadrupedFootstepStatusMessage.class, controllerPubGenerator, s -> {
         QuadrupedFootstepStatusMessage packet = s.takeNextData();
         if (packet.getFootstepStatus() == QuadrupedFootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED)
         {
            RobotQuadrant quadrant = RobotQuadrant.fromByte((byte) packet.getFootstepQuadrant());
            bodyPathMultiplexer.completedFootstep(quadrant, packet);
         }
      });

      MessageTopicNameGenerator controllerSubGenerator = QuadrupedControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);

      controllerStatePublisher = ROS2Tools.createPublisher(ros2Node, HighLevelStateMessage.class, controllerSubGenerator);
      steppingStatePublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedRequestedSteppingStateMessage.class, controllerSubGenerator);
      timedStepListPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedTimedStepListMessage.class, controllerSubGenerator);
      bodyOrientationPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedBodyOrientationMessage.class, controllerSubGenerator);
      bodyTrajectoryPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedBodyTrajectoryMessage.class, controllerSubGenerator);
      bodyHeightPublisher = ROS2Tools.createPublisher(ros2Node, QuadrupedBodyHeightMessage.class, controllerSubGenerator);

      parentRegistry.addChild(registry);
   }

   public void publishTimedStepListToController(QuadrupedTimedStepListMessage message)
   {
      timedStepListPublisher.publish(message);
   }

   public void update()
   {
      limitedDesiredVelocity.update();

      timestamp.set(Conversions.nanosecondsToSeconds(timestampNanos.get()));
      bodyPathMultiplexer.setPlanarVelocityForJoystickPath(limitedDesiredVelocity.getX(), limitedDesiredVelocity.getY(), limitedDesiredVelocity.getZ());
      referenceFrames.updateFrames();

      if (paused.get())
      {
         return;
      }
      else if (xGaitRequested.getValue() && !isInStepState())
      {
         xGaitRequested.set(false);
         stepStream.onEntry();
         sendSteps();
         walking.set(true);
      }
      else if (standingRequested.getBooleanValue())
      {
         standingRequested.set(false);
         requestStopWalking();
         walking.set(false);
      }
      else if (isInBalancingState())
      {
         sendDesiredBodyHeight();

         if (walking.getBooleanValue())
         {
            stepStream.process();
            sendSteps();
         }
         else
         {
            sendDesiredBodyTrajectory();
            //            sendDesiredBodyOrientation();
         }
      }
   }

   public void setDesiredVelocity(double desiredVelocityX, double desiredVelocityY, double desiredVelocityZ)
   {
      this.desiredVelocity.set(desiredVelocityX, desiredVelocityY, desiredVelocityZ);
   }

   public void requestStandPrep()
   {
      HighLevelStateMessage controllerMessage = new HighLevelStateMessage();
      controllerMessage.setHighLevelControllerName(HighLevelStateMessage.STAND_PREP_STATE);
      controllerStatePublisher.publish(controllerMessage);
   }

   public void requestWalkingState()
   {
      HighLevelStateMessage controllerMessage = new HighLevelStateMessage();
      controllerMessage.setHighLevelControllerName(HighLevelStateMessage.STAND_TRANSITION_STATE);
      controllerStatePublisher.publish(controllerMessage);
   }

   private void requestStopWalking()
   {
      QuadrupedRequestedSteppingStateMessage steppingMessage = new QuadrupedRequestedSteppingStateMessage();
      steppingMessage.setQuadrupedSteppingRequestedEvent(QuadrupedSteppingRequestedEvent.REQUEST_STAND.toByte());
      steppingStatePublisher.publish(steppingMessage);
   }

   public void requestXGait()
   {
      xGaitRequested.set(true);
   }

   private boolean isInBalancingState()
   {
      HighLevelStateChangeStatusMessage controllerStateChangeMessage = this.controllerStateChangeMessage.get();
      return (controllerStateChangeMessage != null && controllerStateChangeMessage.getEndHighLevelControllerName() == HighLevelControllerName.WALKING.toByte());
   }

   public boolean isInStepState()
   {
      QuadrupedSteppingStateChangeMessage steppingStateChangeMessage = this.steppingStateChangeMessage.get();
      return isInBalancingState() && (steppingStateChangeMessage != null
            && steppingStateChangeMessage.getEndQuadrupedSteppingStateEnum() == QuadrupedSteppingStateEnum.STEP.toByte());
   }

   public boolean isInStandState()
   {
      QuadrupedSteppingStateChangeMessage steppingStateChangeMessage = this.steppingStateChangeMessage.get();
      return isInBalancingState() && (steppingStateChangeMessage != null
            && steppingStateChangeMessage.getEndQuadrupedSteppingStateEnum() == QuadrupedSteppingStateEnum.STAND.toByte());
   }

   public boolean isWalking()
   {
      return walking.getBooleanValue();
   }

   public void requestStanding()
   {
      standingRequested.set(true);
   }

   private void sendSteps()
   {
      List<? extends QuadrupedTimedStep> steps = stepStream.getSteps();
      List<QuadrupedTimedStepMessage> stepMessages = new ArrayList<>();
      for (int i = 0; i < steps.size(); i++)
         stepMessages.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(steps.get(i)));

      QuadrupedTimedStepListMessage stepsMessage = QuadrupedMessageTools.createQuadrupedTimedStepListMessage(stepMessages, true);
      timedStepListPublisher.publish(stepsMessage);

      QuadrupedBodyOrientationMessage orientationMessage = QuadrupedMessageTools
            .createQuadrupedWorldFrameYawMessage(getPlannedStepsSortedByEndTime(), limitedDesiredVelocity.getZ());
      bodyOrientationPublisher.publish(orientationMessage);
   }

   private final List<QuadrupedTimedOrientedStep> plannedStepsSortedByEndTime = new ArrayList<>();

   private List<QuadrupedTimedOrientedStep> getPlannedStepsSortedByEndTime()
   {
      plannedStepsSortedByEndTime.clear();
      PreallocatedList<QuadrupedTimedOrientedStep> plannedSteps = stepStream.getFootstepPlan().getPlannedSteps();
      plannedStepsSortedByEndTime.addAll(plannedSteps);
      TimeIntervalTools.sortByEndTime(plannedStepsSortedByEndTime);
      return plannedStepsSortedByEndTime;
   }

   public void setDesiredBodyHeight(double desiredBodyHeight)
   {
      this.desiredBodyHeight.set(desiredBodyHeight);
   }

   public void setDesiredBodyOrientation(double yaw, double pitch, double roll, double time)
   {
      desiredOrientationYaw.set(yaw);
      desiredOrientationPitch.set(pitch);
      desiredOrientationRoll.set(roll);
      desiredOrientationTime.set(time);
   }

   public void setDesiredBodyTranslation(double x, double y, double time)
   {
      desiredPositionX.set(x);
      desiredPositionY.set(y);

      desiredOrientationTime.set(time);
   }

   public void setDesiredBodyPose(double x, double y, double yaw, double pitch, double roll, double time)
   {
      desiredPositionX.set(x);
      desiredPositionY.set(y);

      desiredOrientationYaw.set(yaw);
      desiredOrientationPitch.set(pitch);
      desiredOrientationRoll.set(roll);
      desiredOrientationTime.set(time);
   }

   private final FramePoint3D tempPoint = new FramePoint3D();

   private void sendDesiredBodyHeight()
   {
      double bodyHeight = desiredBodyHeight.getAndSet(Double.NaN);

      if (!Double.isNaN(bodyHeight))
      {
         tempPoint.setIncludingFrame(centerFeetZUpFrame, 0.0, 0.0, bodyHeight);
         tempPoint.changeFrame(worldFrame);

         QuadrupedBodyHeightMessage bodyHeightMessage = QuadrupedMessageTools.createQuadrupedBodyHeightMessage(0.0, tempPoint.getZ());
         bodyHeightMessage.setControlBodyHeight(true);
         bodyHeightMessage.setIsExpressedInAbsoluteTime(false);
         bodyHeightPublisher.publish(bodyHeightMessage);
      }
   }

   private void sendDesiredBodyOrientation()
   {
      double desiredYaw = desiredOrientationYaw.getAndSet(Double.NaN);
      double desiredPitch = desiredOrientationPitch.getAndSet(Double.NaN);
      double desiredRoll = desiredOrientationRoll.getAndSet(Double.NaN);
      double desiredTime = desiredOrientationTime.getAndSet(Double.NaN);

      if (!Double.isNaN(desiredYaw))
      {
         offsetBodyOrientationMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().clear();
         offsetBodyOrientationMessage.setIsAnOffsetOrientation(true);
         SO3TrajectoryPointMessage trajectoryPointMessage = offsetBodyOrientationMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().add();
         trajectoryPointMessage.getOrientation().setYawPitchRoll(desiredYaw, desiredPitch, desiredRoll);
         trajectoryPointMessage.setTime(desiredTime);
         bodyOrientationPublisher.publish(offsetBodyOrientationMessage);
      }
   }

   private void sendDesiredBodyTrajectory()
   {
      double desiredX = desiredPositionX.getAndSet(Double.NaN);
      double desiredY = desiredPositionY.getAndSet(Double.NaN);
      double bodyHeight = desiredBodyHeight.getAndSet(Double.NaN);

      double desiredYaw = desiredOrientationYaw.getAndSet(Double.NaN);
      double desiredPitch = desiredOrientationPitch.getAndSet(Double.NaN);
      double desiredRoll = desiredOrientationRoll.getAndSet(Double.NaN);
      double desiredTime = desiredOrientationTime.getAndSet(Double.NaN);

      ReferenceFrame messageFrame = worldFrame;

      SE3TrajectoryMessage se3Trajectory = bodyTrajectoryMessage.getSe3Trajectory();
      se3Trajectory.getTaskspaceTrajectoryPoints().clear();
      se3Trajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(messageFrame));
      se3Trajectory.getAngularSelectionMatrix().setXSelected(false);
      se3Trajectory.getAngularSelectionMatrix().setYSelected(false);
      se3Trajectory.getAngularSelectionMatrix().setZSelected(false);
      se3Trajectory.getLinearSelectionMatrix().setXSelected(false);
      se3Trajectory.getLinearSelectionMatrix().setYSelected(false);
      se3Trajectory.getLinearSelectionMatrix().setZSelected(false);
      SE3TrajectoryPointMessage trajectoryPointMessage = se3Trajectory.getTaskspaceTrajectoryPoints().add();

      if (!Double.isNaN(desiredYaw))
      {
         trajectoryPointMessage.getOrientation().setYawPitchRoll(desiredYaw, desiredPitch, desiredRoll);
         se3Trajectory.getAngularSelectionMatrix().setXSelected(true);
         se3Trajectory.getAngularSelectionMatrix().setYSelected(true);
         se3Trajectory.getAngularSelectionMatrix().setZSelected(true);
      }
      if (!Double.isNaN(bodyHeight))
      {
         tempPoint.setIncludingFrame(centerFeetZUpFrame, 0.0, 0.0, bodyHeight);
         tempPoint.changeFrame(messageFrame);
         trajectoryPointMessage.getPosition().setZ(tempPoint.getZ());
         se3Trajectory.getLinearSelectionMatrix().setZSelected(true);
      }
      if (!Double.isNaN(desiredX) && !Double.isNaN(desiredY))
      {
         tempPoint.setIncludingFrame(centerFeetZUpFrame, desiredX, desiredY, 0.0);
         tempPoint.changeFrame(messageFrame);
         trajectoryPointMessage.getPosition().setX(tempPoint.getX());
         trajectoryPointMessage.getPosition().setY(tempPoint.getY());
         se3Trajectory.getLinearSelectionMatrix().setXSelected(true);
         se3Trajectory.getLinearSelectionMatrix().setYSelected(true);
      }
      trajectoryPointMessage.setTime(desiredTime);
      bodyTrajectoryPublisher.publish(bodyTrajectoryMessage);
   }

   public void setStepSnapper(PointFootSnapper stepSnapper)
   {
      stepStream.setStepSnapper(stepSnapper);
   }

   public QuadrupedXGaitSettingsBasics getXGaitSettings()
   {
      return xGaitSettings;
   }

   public void setShiftPlanBasedOnStepAdjustment(boolean shiftPlanBasedOnStepAdjustment)
   {
      bodyPathMultiplexer.setShiftPlanBasedOnStepAdjustment(shiftPlanBasedOnStepAdjustment);
   }

   public void handleBodyPathPlanMessage(QuadrupedBodyPathPlanMessage bodyPathPlanMessage)
   {
      bodyPathMultiplexer.initialize();
      bodyPathMultiplexer.handleBodyPathPlanMessage(bodyPathPlanMessage);
   }

   public void setPaused(boolean pause)
   {
      paused.set(pause);

      steppingStateChangeMessage.set(null);
      walking.set(false);
   }

   public boolean isPaused()
   {
      return paused.get();
   }

   public QuadrupedReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }

   public Ros2Node getRos2Node()
   {
      return ros2Node;
   }
}